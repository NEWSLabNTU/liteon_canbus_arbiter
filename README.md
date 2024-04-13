# canbus\_arbiter

This ROS node processes control requests from user or Autoware side
and sends CAN bus commands accordingly. It implements vehicle state
initialization, CAN message rate control and driving protection
mechanism.

## File Hierarchy

```
canbus_arbiter
├── __init__.py
├── main.py
├── canbus/
└── state.py
```

- `main.py` is the entry points of the node.
- `canbus/` contains low-level CAN bus messaging code files,
  respectively for throttle, braking, steering and vehicle state
  control.
- `state.py` defines the global state respecting the information from
  CAN bus messages and user control requests.

## Dependency
1. Please install the [Kvaser leaf driver](https://www.kvaser.com/canlib-webhelp/section_install_linux.html).

Steps:
- Download and extract linuxcan directory
- `cd linuxcan`
- `make`
- `sudo make install`
- `sudo make load`
- `lsmod | grep leaf`, if you see leaf, congrats! If not, run next step
- `sudo insmod /lib/modules/$(uname -r)/kernel/drivers/usb/misc/leaf.ko`, `lsmod | grep leaf`, you should see leaf now

2. Install [Python Kvaser canlib](https://pycanlib.readthedocs.io/en/v1.25.393/canlib.html)


## Testing
Before testing, please build the project follow the tutorial [here](https://github.com/NEWSLabNTU/2023-nycu-project/tree/main#build).

Also, please make sure the Kvaser cable is pluged in the PC before running the following commands.
### Keyboard Manual Control
Open a new terminal and run:
```
source ~/autoware/install/setup.bash
ros2 run autoware_manual_control keyboard_control
```
The usage is [here](https://github.com/evshary/autoware_manual_control/tree/928815d9a275097791eabcd7a4c27330dba58e71#usage)

### Canbus arbiter ROS node
Open a new terminal and run:
```
source ~/autoware/install/setup.bash
ros2 run canbus_arbiter canbus_arbiter
```
### Gear
Open a new terminal and run:
```
source ~/autoware/install/setup.bash
ros2 topic echo /gear_status
```

You shall see messages like this
```
stamp:
    sec: 1702144659
    nanosec: 88186326
report: 22
```
Note: `22` means PARKING, `20` means REVERSE, `1` means NEUTRAL, `2` means DRIVE

You can use the keyboard [controller](#Keyboard-Manual-Control) to change the gear manually

### Steering
Open a new terminal and run:
```
source ~/autoware/install/setup.bash
ros2 topic echo /steering_status
```
```
stamp:
    sec: 1702144659
    nanosec: 88186326
steering_tire_angle: 0.0
```
Note: the unit of `steering_tire_angle` is radian, but [keyboard controller](#Keyboard-Manual-Control) uses degree. So, if you see 0.017xxx means 1 degree, 0.034xxx means 2 degree, and so on

You can use the keyboard [controller](#Keyboard-Manual-Control) to change the steering angle manually

### Speed
TODO

### Brake
TODO
