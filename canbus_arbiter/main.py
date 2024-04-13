# Note: fmt: on/off is required by python black formatter

from canlib import canlib
import math,time
from typing import Optional
import rclpy
from rclpy.node import Node

from .state import State
from .constant import Gear, CAN_CHANNEL, THROTTLE_STOP_DELTA, FULL_STOP_DELTA, DEFAULT_BRAKE_MAP, DEFAULT_ACCEL_MAP
from .utils import clamp

from .read import MsgType, read_msg
from .brake import send_message as send_brake_cmd
from .gear import (
    send_gear_command as send_gear_cmd,
    get_stall_status as get_gear_status,
)
from .velocity import (
    send_velocity_command as send_velocity_cmd,
    get_acceleration,
    get_velocity,
    #get_Throttle_Pedal_signal as get_throttle_pedal_signal,
)
from .wheel import send_angle_message as send_angle_cmd, get_wheeling_angle

from tier4_control_msgs.msg import GateMode
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import GearCommand

from autoware_auto_vehicle_msgs.msg import GearReport
from autoware_auto_vehicle_msgs.msg import SteeringReport
from autoware_auto_vehicle_msgs.msg import VelocityReport

class CanbusArbiter(Node):
    def __init__(self):
        super().__init__("canbus_arbiter")
        self.declare_parameter("accel_map", DEFAULT_ACCEL_MAP) # see launch file
        self.declare_parameter("brake_map", DEFAULT_BRAKE_MAP) # see launch file
        self.declare_parameter("control", "simulate") # see launch file

        self.ackermann_cmd_topic = None
        self.gear_cmd_topic = None

        # can bus
        if self.get_parameter('control').get_parameter_value().string_value == "manual":
            self.bitrate = canlib.canBITRATE_500K
            self.bitrateFlags = canlib.canDRIVER_NORMAL
            self.ch = canlib.openChannel(channel=CAN_CHANNEL)
            self.ch.setBusOutputControl(self.bitrateFlags)
            self.ch.setBusParams(self.bitrate)
            self.ch.busOn()
        else:
            self.ch = None

        if self.get_parameter('control').get_parameter_value().string_value == "manual":
            self.ackermann_cmd_topic = "/external/selected/control_cmd"
            self.gear_cmd_topic = "/external/selected/gear_cmd"
        else: # simulate and auto
            self.ackermann_cmd_topic = "/control/command/control_cmd"
            self.gear_cmd_topic = "/control/command/gear_cmd"

        # Initialize the state
        self.state = State()
        #self.can_callback()
        #if not self.is_vehicle_ok():
        #    quit()

        # Create subscribers
        self.control_cmd_sub = self.create_subscription(
            AckermannControlCommand,
            self.ackermann_cmd_topic,
            self.control_cmd_callback,
            10,
        )
        self.gear_control_cmd_sub = self.create_subscription(
            GearCommand,
            self.gear_cmd_topic,
            self.gear_control_cmd_callback,
            10,
        )
        self.gatemode_cmd_sub = self.create_subscription(
            GateMode, "/control/current_gate_mode", self.gatemode_cmd_callback, 10
        )

        # prevent unused variable warning
        self.control_cmd_sub
        self.gear_control_cmd_sub
        self.gatemode_cmd_callback

        # Create publishers
        self.speed_pub = self.create_publisher(
            VelocityReport,
            # "/vehicle/status/velocity_status", Autoware topic
            "velocity_status",
            10,
        )
        self.gear_pub = self.create_publisher(
            GearReport,
            # "/vehicle/status/gear_status", Autoware topic
            "gear_status",
            10,
        )
        self.steering_pub = self.create_publisher(
            SteeringReport,
            # "/vehicle/status/steering_status", Autoware topic
            "steering_status",
            10,
        )

        # Create a periodic task
        timer_period_secs = 0.05  # seconds
        self.timer = self.create_timer(timer_period_secs, self.timer_callback)

        # Create a periodic task for reading the status of vehicle
        can_callback_period_secs = 0.01
        self.vehicle_status_timer = self.create_timer(
            can_callback_period_secs, self.can_callback
        )


    def step(self) -> None:
        # self.get_logger().info('{}'.format(self.state))
        # simple on-off gear controller

        if self.get_parameter('control').get_parameter_value().string_value == "simulate":
            state_value_map = dict()
            state_value_map['speed'] = self.state.target_speed
            state_value_map['steering'] = self.state.target_steering_angle
            state_value_map['gear'] = self.state.target_gear
            state_value_map['gatemode'] = self.state.target_mode

            for state, value in state_value_map.items():
                self.get_logger().info("target {}: {}".format(state, value))

        # simple on-off mode controller
        if self.state.current_mode != self.state.target_mode:
            # Note: Brake and gear changes must be done manually; only EPS and throttle can be changed via CAN message.
            if self.ch is None:
                return

            if self.state.target_mode == GateMode.EXTERNAL:
                self.state.current_mode = GateMode.EXTERNAL
                send_angle_cmd(self.ch, self.state.current_steering_angle, 1)
                send_velocity_cmd(self.ch, self.state.current_throttle, 1)
                send_brake_cmd(self.ch, 0)
            else:
                self.state.current_mode = GateMode.AUTO
                send_angle_cmd(self.ch, self.state.current_steering_angle, 0)
                send_velocity_cmd(self.ch, self.state.current_throttle, 0)
                send_brake_cmd(self.ch, 0)

        if abs_equal(self.state.target_speed, 0.0) and not abs_equal(self.state.previous_speed, 0.0):
            #print("target speed: ", self.state.target_speed)
            #print("previous speed: ", self.state.previous_speed)
            if self.ch is None:
                return

            send_brake_cmd(self.ch, 15)
            return

        # P steering angle controller
        if self.state.current_gear == Gear.DRIVE.value or \
            self.state.current_gear == Gear.REVERSE.value:
            # steering_angle = self.steering_controller(self.state.current_steering_angle)
            if self.ch is None:
                return
            #print("Send angle command")
            send_angle_cmd(self.ch, self.state.target_steering_angle, 1)
            # self.state.current_steering_angle += steering_angle # for offline debuging

        if self.state.current_gear != self.state.target_gear and \
                abs_equal(self.state.current_speed, 0.0):
            gear = self.gearval_2_char(self.state.target_gear)
            if gear is None:
                self.get_logger().warning("change gear failed: invalid gear")
            else:
                if self.ch is None:
                    return
                send_gear_cmd(self.ch, gear)
                #self.state.current_gear = self.gearchar_2_val(gear) # for offline debuging

        #print("target speed: {}".format(self.state.target_speed))
        if self.state.current_gear == Gear.DRIVE.value and self.state.target_speed <= self.state.steady_speed:
            return

        if self.state.previous_target_speed == self.state.target_speed:
            #print("same target return")
            return

        if self.state.target_speed != 0 and \
                (self.state.current_gear == Gear.DRIVE.value or \
                self.state.current_gear == Gear.REVERSE.value):
                if self.state.current_speed >= self.state.steady_speed:
                    if self.ch is None:
                        return
                    send_velocity_cmd(self.ch, 0.0, 1)
                    send_brake_cmd(self.ch, 0)
                    return
                elif self.state.current_speed < self.state.steady_speed:
                    if self.ch is None:
                        return
                    send_velocity_cmd(self.ch, self.state.min_throttle, 1)
                    send_brake_cmd(self.ch, 0)
                # self.state.current_throttle += self.state.throttle_delta # for offline debugging

    # Send CAN commands and publish ROS messages according to the current state.
    def timer_callback(self) -> None:
        if not self.is_vehicle_ok():
            # reset before quit?
            quit()

        self.vehicle_status_report()

        # TODO: more protection?

        self.step()

    def convert_to_target_steering_angle(self, target_steering_angle: float) -> float:
        target_steering_angle = -target_steering_angle
        target_steering_angle = clamp(
            target_steering_angle, self.state.max_steering_angle
        )
        return target_steering_angle

    def convert_to_target_speed(self, target_speed: float) -> float:
        target_speed = clamp(target_speed, self.state.max_speed)
        return target_speed

    def convert_to_target_accel(self, target_accel):
        if abs(self.state.target_speed) < FULL_STOP_DELTA:
            return -self.state.max_decel
        elif target_accel < -self.state.max_decel:
            return -self.state.max_decel
        else:
            return self.state.max_accel

    def convert_to_target_throttle(self, target_throttle):
        if abs(self.state.target_speed) < FULL_STOP_DELTA:
            return 0.0
        target_throttle = clamp(target_throttle, self.state.max_throttle)
        return target_throttle

    # TODO: Process received control request from ROS/Autoware
    def control_cmd_callback(self, msg: AckermannControlCommand) -> None:
        # if self.state.current_mode == GateMode.AUTO:
        #    # self.get_logger().info('control failed: AUTO mode is not listening control commands')
        #    return

        lateral = msg.lateral
        longitudinal = msg.longitudinal
        # print(lateral)
        # print(longitudinal)

        self.state.previous_target_speed = self.state.target_speed
        self.state.target_speed = self.convert_to_target_speed(longitudinal.speed)

        self.state.target_steering_angle = self.convert_to_target_steering_angle(
            self.rad_2_deg(lateral.steering_tire_angle)
        )
        self.steering_controller.setpoint = self.state.target_steering_angle

    def gear_control_cmd_callback(self, msg: GearCommand) -> None:
        target_gear = int(msg.command)  # it is a num string before casting

        # FIXME: target gear should be given from upper control
        if not abs_equal(self.state.target_speed, 0.0):
            target_gear = Gear.DRIVE.value
            if self.ch is None:
                return
            send_brake_cmd(self.ch, 0) # FIXME: second speed up cannot work without this

        # if self.state.current_mode == GateMode.AUTO:
        #    # self.get_logger().info('change gear failed: AUTO mode is not listening gear commands')
        #    return

        if self.state.target_gear == target_gear:
            # self.get_logger().info('gear is not changed')
            return

        # only four gears are available
        if (
            target_gear != Gear.PARKING.value
            and target_gear != Gear.REVERSE.value
            and target_gear != Gear.NEUTRAL.value
            and target_gear != Gear.DRIVE.value
        ):
            self.get_logger().warning("change gear failed: invalid gear")
            return

        if not abs_equal(self.state.current_speed, 0.0):  # moving vehicle cannot change gear
            self.get_logger().warning(
                "change gear failed: moving vehicle cannot change gear"
            )
            return

        # self.get_logger().info('change target gear from %s to %s' % \
        #        (self.gearval_2_str(self.state.target_gear), self.gearval_2_str(target_gear)))
        self.state.target_gear = target_gear

    def gatemode_cmd_callback(self, msg: GateMode) -> None:
        gatemode = int(msg.data)  # it is a num string before casting
        # print(gatemode)

        if gatemode != GateMode.AUTO and gatemode != GateMode.EXTERNAL:
            self.get_logger().warning("change gate mode failed: invalid gate mode")
            return

        self.state.target_mode = gatemode

    def kmh_speed_2_ms_speed(self, km_per_h_speed):
        return (km_per_h_speed * 5.0) / 18

    def can_callback(self) -> None:
        if self.ch is None:
            # print("channel is None")
            return

        ret = read_msg(self.ch)
        if ret == None:
            return

        msg, val = ret

        if msg == MsgType.THROTTLE_PEDAL:
            throttle = val
            # return throttle is 0 - 100, but we want 0 - 1000
            self.state.current_throttle = throttle * 10
        elif msg == MsgType.BRAKE_PEDAL:
            print("brake pedal value: ", val)
        elif msg == MsgType.ACCELERATION:
            accel = val
            self.state.current_accel = accel
        elif msg == MsgType.VELOCITY:
            speed = val
            self.state.previous_speed = self.state.current_speed
            self.state.current_speed = speed
        elif msg == MsgType.WHEEL_ANGLE:
            steering_angle = val
            self.state.current_steering_angle = steering_angle
        elif msg == MsgType.GEAR:
            gear = self.gearchar_2_val(val)
            self.state.current_gear = gear

        # fmt: off
        #TODO: read from canlib
        steering_ADAS                       = False #FIXME: read status from canbus
        throttle_ADAS                       = False #FIXME: read status from canbus
        #brake_ADAS                          = False # no brake ADAS status feedback from CAN Bus to determine if ADAS mode is on
        #gear_ADAS                           = False # no gear ADAS status feedback from CAN Bus to determine if ADAS mode is on
        current_wave_center_compensate_diag = False
        sensor_5_volt_monitor_diag          = False
        steering_sensor_diag                = False
        current_sensor_diag                 = False
        motor_inhibit_diag                  = False
        eps_status_diag                     = False
        phase_current_A_center_diag         = False
        phase_current_A_range_diag          = False
        phase_current_B_center_diag         = False
        phase_current_B_range_diag          = False
        hall_sensor_diag                    = False
        hall_ABC_sum_diag                   = False
        encoder_sensor_diag                 = False
        position_QEP_diag                   = False
        brake_controller_diag               = True
        brake_diag_code                     = False
        # fmt: on

        # All ADAS modes need to be enabled to enter AUTONOMOUS mode
        if not (steering_ADAS and throttle_ADAS):
            self.state.current_mode = GateMode.EXTERNAL
            # self.get_logger().info('current gate mode: external(manual)')
        else:
            self.state.current_mode = GateMode.AUTO
            # self.get_logger().info('current gate mode: autonomous')

        # fmt: off
        self.state.current_wave_center_compensate_diag  = current_wave_center_compensate_diag
        self.state.sensor_5_volt_monitor_diag           = sensor_5_volt_monitor_diag
        self.state.steering_sensor_diag                 = steering_sensor_diag
        self.state.current_sensor_diag                  = current_sensor_diag
        self.state.motor_inhibit_diag                   = motor_inhibit_diag
        self.state.eps_status_diag                      = eps_status_diag
        self.state.phase_current_A_center_diag          = phase_current_A_center_diag
        self.state.phase_current_A_range_diag           = phase_current_A_range_diag
        self.state.phase_current_B_center_diag          = phase_current_B_center_diag
        self.state.phase_current_B_range_diag           = phase_current_B_range_diag
        self.state.hall_sensor_diag                     = hall_sensor_diag
        self.state.hall_ABC_sum_diag                    = hall_ABC_sum_diag
        self.state.encoder_sensor_diag                  = encoder_sensor_diag
        self.state.position_QEP_diag                    = position_QEP_diag
        self.state.brake_controller_diag                = brake_controller_diag
        self.state.brake_diag_code                      = brake_diag_code
        # fmt: on


    def is_vehicle_ok(self) -> bool:
        if self.state.current_wave_center_compensate_diag:
            self.get_logger().error("current wave center compensate failed")
            return False
        if self.state.sensor_5_volt_monitor_diag:
            self.get_logger().error("sensor 5 volt monitor failed")
            return False
        if self.state.steering_sensor_diag:
            self.get_logger().error("steering sensor failed")
            return False
        if self.state.current_sensor_diag:
            self.get_logger().error("current sensor failed")
            return False
        if self.state.motor_inhibit_diag:
            self.get_logger().error("motor inhibit failed")
            return False
        if self.state.eps_status_diag:
            self.get_logger().error("EPS status failed")
            return False
        if self.state.phase_current_A_center_diag:
            self.get_logger().error("phase current A center failed")
            return False
        if self.state.phase_current_A_range_diag != 0:
            self.get_logger().error("phase current A range failed")
            return False
        if self.state.phase_current_B_center_diag:
            self.get_logger().error("phase current B center failed")
            return False
        if self.state.phase_current_B_range_diag:
            self.get_logger().error("phase current B range failed")
            return False
        if self.state.hall_sensor_diag:
            self.get_logger().error("hall sensor failed")
            return False
        if self.state.hall_ABC_sum_diag:
            self.get_logger().error("hall ABC sum failed")
            return False
        if self.state.encoder_sensor_diag:
            self.get_logger().error("encoder sensor failed")
            return False
        if self.state.position_QEP_diag:
            self.get_logger().error("position QEP failed")
            return False
        if not self.state.brake_controller_diag:
            self.get_logger().error("brake controller failed")
            return False
        if self.state.brake_diag_code:
            self.get_logger().error("brake code failed")
            return False

        return True

    def rad_2_deg(self, rad):
        return round((rad * 180) / math.pi)

    def deg_2_rad(self, deg):
        return (deg * math.pi) / 180

    def gearval_2_str(self, gear) -> Optional[str]:
        if gear == Gear.PARKING.value:
            return Gear.PARKING.name
        elif gear == Gear.REVERSE.value:
            return Gear.REVERSE.name
        elif gear == Gear.NEUTRAL.value:
            return Gear.NEUTRAL.name
        elif gear == Gear.DRIVE.value:
            return Gear.DRIVE.name
        else:
            return None

    def gearval_2_char(self, gear) -> Optional[str]:
        if gear == Gear.PARKING.value:
            return "P"
        elif gear == Gear.REVERSE.value:
            return "R"
        elif gear == Gear.NEUTRAL.value:
            return "N"
        elif gear == Gear.DRIVE.value:
            return "D"
        else:
            return None

    def gearchar_2_val(self, gear) -> Optional[int]:
        if gear == "P":
            return Gear.PARKING.value
        elif gear == "R":
            return Gear.REVERSE.value
        elif gear == "N":
            return Gear.NEUTRAL.value
        elif gear == "D":
            return Gear.DRIVE.value
        else:
            return None

    def vehicle_status_report(self) -> None:
        gear_report = GearReport()
        gear_report.stamp = self.get_clock().now().to_msg()
        gear_report.report = self.state.current_gear
        self.gear_pub.publish(gear_report)

        steering_report = SteeringReport()
        steering_report.stamp = self.get_clock().now().to_msg()
        steering_report.steering_tire_angle = self.deg_2_rad(
            self.state.current_steering_angle
        )
        self.steering_pub.publish(steering_report)

        speed_report = VelocityReport()
        speed_report.header.stamp = self.get_clock().now().to_msg()
        speed_report.header.frame_id = "speed"
        speed_report.longitudinal_velocity = float(self.state.current_speed)
        speed_report.lateral_velocity = 0.0  # we need this?
        speed_report.heading_rate = 0.0  # we need this?
        self.speed_pub.publish(speed_report)


def abs_equal(lhs, rhs) -> bool:
    return abs((lhs - rhs)) <= 1e-5

def main():
    try:
        rclpy.init()
        node = CanbusArbiter()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("Manually terminated.")
    finally:
        if node.ch is None:
            return
        send_gear_cmd(ch=node.ch, gear="P")
        send_angle_cmd(ch=node.ch, angle=0, active=0)
        send_velocity_cmd(ch=node.ch, velocity=0, active=0)
        send_brake_cmd(ch=node.ch, deceleration=0)
        time.sleep(0.5)
        node.ch.busOff()
        node.ch.close()

if __name__ == "__main__":
    main()
