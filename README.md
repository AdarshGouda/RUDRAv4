# RUDRAv4 ROS2 Workspace

RUDRAv4 is the ROS2 upgrade path for the RUDRA rover. The current base-control flow keeps the existing hardware split:

```text
PS2 wireless controller
  ↓
Arduino Uno with PS2 receiver
  ↓ USB serial: J,select,ry,rx,ly,lx
NUC rudra running ROS2 Lyrical
  ↓ USB serial: D,left,right,enable
Teensy 4.1
  ↓ Serial1/TX1 single-wire packet serial, 9600 baud
2x Sabertooth drivers
  ↓
RUDRA rover motors
```

## Preserved RUDRAv3 hardware assumptions

| Item | Value |
|---|---|
| PS2 Uno pins | CLK=9, CMD=7, ATT=6, DAT=8 |
| Right Sabertooth | Address 128 |
| Left Sabertooth | Address 129 |
| Sabertooth serial | Teensy Serial1 / TX1, 9600 baud |
| Wheelbase | 0.29 m |
| Wheel radius reference | 0.0675 m |
| Manual mixer | throttle from Ly, steering from Rx, left=throttle-steering, right=throttle+steering |

## Build

```bash
cd /home/rudra/Projects/RUDRAv4
bash scripts/install_deps.sh
bash scripts/build_ws.sh
source /opt/ros/lyrical/setup.bash
source install/setup.bash
```

## Configure serial ports

Find persistent USB serial paths:

```bash
bash scripts/list_serial.sh
```

Edit:

```bash
nano src/rudra_base_bridge/config/rudra_v4_hardware.yaml
```

Set:

```yaml
uno_port: "/dev/serial/by-id/YOUR_UNO_PORT"
teensy_port: "/dev/serial/by-id/YOUR_TEENSY_PORT"
```

Use `/dev/serial/by-id/...` instead of `/dev/ttyACM0` when possible.

## Run PS2 manual drive bridge

```bash
bash scripts/run_ps2_bridge.sh
```

For battery bringup and headless checks, open:

```text
docs/startup.html
```

Useful monitors:

```bash
ros2 topic echo /rudra/ps2_raw
ros2 topic echo /rudra/drive_cmd
ros2 topic echo /rudra/teensy_ack
ros2 topic echo /cmd_vel
```

## Run normal `/cmd_vel` bridge

Use this later for keyboard teleop, joystick teleop, SLAM, or Nav2. Do not run it at the same time as the PS2 bridge because both nodes would try to own the Teensy serial port.

```bash
bash scripts/run_cmd_vel_bridge.sh
```

## Firmware

Plain-serial firmware examples are included:

```text
src/rudra_base_bridge/firmware/uno_ps2_plain_serial/uno_ps2_plain_serial.ino
src/rudra_base_bridge/firmware/teensy_sabertooth_serial_controller/teensy_sabertooth_serial_controller.ino
```

These replace the old ROS1 `rosserial` path for ROS2 bringup. The Uno only publishes PS2 readings to the NUC. The Teensy is the only board that sends packet serial to the Sabertooths.

## Serial protocols

Uno to NUC:

```text
J,select,ry,rx,ly,lx
```

NUC to Teensy:

```text
D,left,right,enable
```

`left` and `right` use the Sabertooth command range `-127..127`.
