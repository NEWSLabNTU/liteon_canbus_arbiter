from dataclasses import dataclass
from .constant import Gear

@dataclass
class State:
    previous_speed:         float = 0.0
    current_speed:          float = 0.0
    previous_target_speed:           float = 0.0
    target_speed:           float = 0.0

    target_accel:             float = 0.0
    current_accel:            float = 0.0

    target_steering_angle:  float = 0.0
    current_steering_angle: float = 0.0

    target_gear:            int = Gear.PARKING.value   # default should be PARKING
    current_gear:           int = Gear.PARKING.value   # default should be PARKING

    target_throttle:        float = 0.0
    current_throttle:       float = 0.0

    target_mode:            int = 0
    current_mode:           int = 0

    # TODO: more fields

    # diagnostic refers to GateWayCANmessageMap
    # Note: current here is not same as above current
    current_wave_center_compensate_diag: bool = False
    sensor_5_volt_monitor_diag:          bool = False
    steering_sensor_diag:                bool = False
    current_sensor_diag:                 bool = False
    motor_inhibit_diag:                  bool = False
    eps_status_diag:                     bool = False
    phase_current_A_center_diag:         bool = False
    phase_current_A_range_diag:          bool = False
    phase_current_B_center_diag:         bool = False
    phase_current_B_range_diag:          bool = False
    hall_sensor_diag:                    bool = False
    hall_ABC_sum_diag:                   bool = False
    encoder_sensor_diag:                 bool = False
    position_QEP_diag:                   bool = False
    brake_controller_diag:               bool = True
    brake_diag_code:                     bool = False
    # TODO: more diagnostic fields

    # max_speed: float = 14              # default: 20  km/h 
    # min_throttle: float = 200.0          # default: 20  km/h 

    # approximate 7-9%
    min_throttle: float = 150.0          # default: 20  km/h 
    max_steering_angle: float = 550    # default: 50  degrees
    max_speed: float = 100.0             # default: 20  km/h
    #max_accel: float = 1.0             # default: 1.0 m/s^2
    #max_decel: float = 3.0             # default: 3.0 m/s^2
    max_throttle: float = 70.0          # default: 7%
    # TODO: more restriction fields
