from canlib import canlib, Frame
import argparse
from typing import List
from canlib.canlib import Channel
import time

def get_acceleration(ch: Channel) -> float:
    acceleration = None
    while True:
        frame = ch.read(timeout=10)  # 10ms
        if frame.id == 96:
            byte0 = frame.data[0]
            byte1 = frame.data[1]
            acceleration = ((byte0 + byte1 * 256) * 0.01) - 100
            break

    if acceleration is not None:
        return acceleration
    else:
        raise ValueError("No acceleration received.")


def get_velocity(ch: Channel) -> float:
    velocity = None
    while True:
        frame = ch.read(timeout=10)  # 10ms
        if frame.id == 97:
            byte0 = frame.data[0]
            velocity = byte0
            break

    if velocity is not None:
        return velocity
    else:
        raise ValueError("No velocity received.")


def get_Throttle_Pedal(ch: Channel) -> float:
    th_pedal_signal = None
    while True:
        frame = ch.read(timeout=10)  # 10ms
        if frame.id == 80:
            byte0 = frame.data[0]
            th_pedal_signal = byte0 * 0.3906
            break

    if th_pedal_signal is not None:
        return th_pedal_signal
    else:
        raise ValueError("No throttle pedal received.")


def mode_switch(mode: int) -> int:
    if mode == 1:
        print("===========ADAS Control mode===========")
        return 1
    else:
        print("===========Pedal mode===========")
        return 0


def velocity_to_bytes(velocity: int) -> List[int]:
    value = int(velocity)
    byte0 = value % 256
    byte1 = value // 256
    return [byte0, byte1]


def send_velocity_command(ch: Channel, velocity: int, active: int):
    velocity_byte = velocity_to_bytes(velocity)

    frame_data = velocity_byte + [
        active,
        0,
        0,
        0,
        0,
        0,
    ]
    frame = Frame(id_=117, data=frame_data)
    ch.write(frame)


def main() -> None:
    parser = argparse.ArgumentParser(description="Control CAN bus angle message.")
    parser.add_argument("--channel", type=int, default=0, help="CAN channel number")
    parser.add_argument(
        "--velocity", type=int, default=0, help="Initial velocity to send, range:0~1000"
    )
    parser.add_argument(
        "--mode", type=int, default=0, help="0 for EPS mode, 1 for AngleControl mode"
    )
    args = parser.parse_args()

    bitrate = canlib.canBITRATE_500K
    bitrateFlags = canlib.canDRIVER_NORMAL
    ch = canlib.openChannel(channel=args.channel)
    ch.setBusOutputControl(bitrateFlags)
    ch.setBusParams(bitrate)
    ch.busOn()

    try:
        while True:
            active = mode_switch(mode=args.mode)
            send_velocity_command(ch=ch, velocity=args.velocity, active=active)
            time.sleep(0.001)
    except KeyboardInterrupt:
        print("Manually terminated.")
    finally:
        send_velocity_command(ch=ch, velocity=0, active=0)
        time.sleep(0.002)
        ch.busOff()
        ch.close()


if __name__ == "__main__":
    main()
