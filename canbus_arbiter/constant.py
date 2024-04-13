from enum import Enum

CAN_CHANNEL = 0

THROTTLE_STOP_DELTA = 0.01
FULL_STOP_DELTA = 0.00001

DEFAULT_ACCEL_MAP = "~/autoware/src/universe/autoware.universe/vehicle/raw_vehicle_cmd_converter/data/default/accel_map.csv"
DEFAULT_BRAKE_MAP = "~/autoware/src/universe/autoware.universe/vehicle/raw_vehicle_cmd_converter/data/default/brake_map.csv"

class Gear(Enum):
    PARKING = 22
    REVERSE = 20
    NEUTRAL = 1
    DRIVE = 2
