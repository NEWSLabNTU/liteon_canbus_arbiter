from typing import List
from canlib import canlib, Frame
import time
import argparse
from canlib.canlib import Channel


def get_wheeling_angle(ch: Channel) -> float:
    angle = None

    while True:
        frame = ch.read(timeout=10)  # 10ms
        if frame.id == 103:
            byte2 = frame.data[2]
            byte3 = frame.data[3]
            angle = (byte2 + (byte3 * 256)) * 0.1 - 800
            break

    if angle is not None:
        return angle
    else:
        raise ValueError("No wheeling angle received.")


def mode_switch(mode: int) -> int:
    if mode == 1:
        print("===========AngleControl mode===========")
        return 1
    else:
        print("===========EPS mode===========")
        return 0


def angle_to_bytes(angle: float) -> List[int]:
    value = int((angle + 800) * 10)
    byte0 = value % 256
    byte1 = value // 256
    return [byte0, byte1]  # [0, 1]


def send_angle_message(ch: Channel, angle: float, active: int):
    data = angle_to_bytes(angle)
    data = data + [0, 0, active, 0, 0, 0]
    frame = Frame(id_=101, data=data)
    ch.write(frame)


def main() -> None:
    parser = argparse.ArgumentParser(description="Control CAN bus angle message.")
    parser.add_argument("--channel", type=int, default=0, help="CAN channel number")
    parser.add_argument(
        "--angle",
        type=float,
        default=0.0,
        help="Initial angle to send, range:-550 ~ 550",
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
            send_angle_message(ch=ch, angle=args.angle, active=active)
            time.sleep(0.001)
    except KeyboardInterrupt:
        print("Manually terminated.")
    finally:
        send_angle_message(ch=ch, angle=0, active=0)
        time.sleep(0.002)
        ch.busOff()
        ch.close()


if __name__ == "__main__":
    main()
