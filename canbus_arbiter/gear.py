from canlib import canlib, Frame
import time
import argparse
from canlib.canlib import Channel


def get_stall_status(ch: Channel) -> str:
    byte2 = None

    while True:
        frame = ch.read(timeout=10)  # 10ms
        if frame.id == 361:
            byte2 = frame.data[2]
            break

    if byte2 is not None:
        if byte2 == 0:
            return "P"
        elif byte2 == 1:
            return "R"
        elif byte2 == 2:
            return "N"
        elif byte2 == 3:
            return "D"
        else:
            raise ValueError("Invalid gear value. Should be [0,1,2,3]")
    else:
        raise ValueError("No gear value received.")


def send_gear_command(ch: Channel, gear: str):
    gear_mapping = {"P": 0x00, "R": 0x01, "N": 0x02, "D": 0x03}
    print("gear:",gear)
    if gear not in gear_mapping:
        raise ValueError("Invalid gear value. Choose from ['P', 'R', 'N', 'D']")

    frame_data = [gear_mapping[gear]] + [0, 0, 0, 0, 0, 0, 0]
    frame = Frame(id_=360, data=frame_data)
    ch.write(frame)
    print("sdsdsd")

def main() -> None:
    parser = argparse.ArgumentParser(description="Control CAN bus angle message.")
    parser.add_argument("--channel", type=int, default=0, help="CAN channel number")
    parser.add_argument(
        "--gear", type=str, default="P", help="Initial gear to send, e.g. P,R,N,D"
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
            send_gear_command(ch=ch, gear=args.gear)
            time.sleep(0.001) #avoid buffer overflow
    except KeyboardInterrupt:
        print("Manually terminated.")
    finally:
        # print(ch)
        send_gear_command(ch=ch, gear="P")
        time.sleep(0.001)
        ch.busOff()
        ch.close()


if __name__ == "__main__":
    main()
