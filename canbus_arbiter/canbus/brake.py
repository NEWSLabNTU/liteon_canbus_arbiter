from canlib import canlib, Frame
from canlib.canlib import Channel
import argparse
import time


def get_brake_pedal(ch: Channel) -> bool:
    byte0 = None

    while True:
        frame = ch.read(timeout=10)  # 10ms
        if frame.id == 81:
            byte0 = frame.data[0]
            break

    if byte0 is not None:
        return byte0
    else:
        raise ValueError("No brake pedal signal received.")


def deceleration_to_byte(deceleration: float) -> int:
    return int(deceleration)


def send_message(ch: Channel, deceleration: float):
    deceleration_byte = deceleration_to_byte(deceleration)
    frame1_data = [deceleration_byte] + [0, 0, 0, 0, 0, 0, 0]
    frame1 = Frame(id_=104, data=frame1_data)
    ch.write(frame1)


def main() -> None:
    parser = argparse.ArgumentParser(description="Control CAN bus brake message.")
    parser.add_argument("--channel", type=int, default=0, help="CAN channel number")
    parser.add_argument(
        "--deceleration",
        type=float,
        default=0,
        help="Initial deceleration to send, range:0 ~ 90",
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
            send_message(ch=ch, deceleration=args.deceleration)
            time.sleep(0.001)
    except KeyboardInterrupt:
        print("Manually terminated.")
    finally:
        send_message(ch=ch, deceleration=0)
        time.sleep(0.002)
        ch.busOff()
        ch.close()


if __name__ == "__main__":
    main()
