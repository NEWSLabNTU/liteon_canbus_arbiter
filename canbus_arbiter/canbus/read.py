from canlib import canlib, Frame
from canlib.canlib import Channel
from enum import Enum
import argparse
import time

class MsgType(Enum):
    THROTTLE_PEDAL = 1
    BRAKE_PEDAL = 2
    ACCELERATION = 3
    VELOCITY = 4
    WHEEL_ANGLE = 5
    GEAR = 6


def read_msg(ch: Channel):
    frame = ch.read()
    if frame.id == 80:
        byte0 = frame.data[0]
        th_pedal_signal = byte0 * 0.3906
        return MsgType.THROTTLE_PEDAL, th_pedal_signal
        
    elif frame.id == 81:  # pedal
        pedal = frame.data[0]
        return MsgType.BRAKE_PEDAL, pedal

    elif frame.id == 96:
        byte0 = frame.data[0]
        byte1 = frame.data[1]
        acceleration = ((byte0 + byte1 * 256) * 0.01) - 100
        return MsgType.ACCELERATION, acceleration

    elif frame.id == 97:
        velocity = frame.data[0]
        return MsgType.VELOCITY, velocity

    elif frame.id == 103:
        byte2 = frame.data[2]
        byte3 = frame.data[3]
        angle = (byte2 + (byte3 * 256)) * 0.1 - 800
        return MsgType.WHEEL_ANGLE, angle

    elif frame.id == 361:
        byte2 = frame.data[2]
        gear = None

        if byte2 == 0:
            gear = "P"
        elif byte2 == 1:
            gear = "R"
        elif byte2 == 2:
            gear = "N"
        elif byte2 == 3:
            gear = "D"
        else:
            raise ValueError("Invalid gear value. Should be [0,1,2,3]")

        return MsgType.GEAR, gear



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
            ret = read_msg(ch)
            if ret is not None:
                msg_type, value = ret
                print("{}\t{}".format(msg_type, value))
            time.sleep(0.001) #avoid buffer overflow
    except KeyboardInterrupt:
        print("Manually terminated.")
    finally:
        # print(ch)
        ch.busOff()
        ch.close()
    
if __name__ == '__main__':
    main()
