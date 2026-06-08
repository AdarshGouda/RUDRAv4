# RUDRAv4 ROS2 Workspace

RUDRAv4 is the ROS2 upgrade path for the RUDRA rover. The current base-control flow keeps the existing hardware split:

```text
PS2 wireless controller
  ↓
Arduino Uno with PS2 receiver
  ↓ USB serial: J,select,ry,rx,ly,lx,guard
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

Use this for the normal PS2-only Rudra bringup:

```bash
bash scripts/run_ps2_bridge.sh
```

If you want the PS2 bridge and LiDAR obstacle guard in the same launch, use:

```bash
bash scripts/run_ps2_lidar_bridge.sh /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

Those wrappers call the new launch files directly:

- `rudra_ps2.launch.py` for the normal PS2-only bringup.
- `rudra_ps2_lidar.launch.py` for the PS2 plus LiDAR obstacle bringup.
- `cmd_vel_to_teensy.launch.py` for ROS `/cmd_vel` control to the Teensy.
- `lidar.launch.py` for LiDAR-only `/scan` publishing.
- `lidar_view.launch.py` for RViz-only viewing of `/scan` and the obstacle marker.

On Aghora, you can visualize `/scan` in RViz with the packaged view launch:

```bash
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
source install/setup.bash
ros2 launch rudra_base_bridge lidar_view.launch.py
```

`lidar_view.launch.py` starts RViz only. It does not start the LiDAR driver or
any drive bridge.

The helper scripts in `scripts/` now default to `ROS_DOMAIN_ID=42` and
`ROS_LOCALHOST_ONLY=0` when those variables are unset, so Rudra and Aghora stay
on the same DDS domain by default.

Open the main system manual:

```text
docs/rudrav4-system-manual.html
```

Useful monitors:

```bash
ros2 topic echo /rudra/ps2_raw
ros2 topic echo /rudra/drive_cmd
ros2 topic echo /rudra/teensy_ack
ros2 topic echo /rudra/obstacle_guard
ros2 topic echo /cmd_vel
```

## LiDAR obstacle guard

Both bridge nodes subscribe to `/scan`, matching the RUDRAv3 navigation
LaserScan convention. Forward motion is scaled down when an obstacle is inside
the configured front cone and is blocked inside the stop distance. Reverse and
turn-in-place commands remain available so the rover can back out or rotate.

The obstacle guard only needs a valid `/scan`; on this robot that comes from a
`YDLIDAR G2B` through `YDLidar-SDK` plus `ydlidar_ros2_driver`. The working
RUDRAv4 setup uses the LiDAR adapter's `/dev/serial/by-id/...` path at `128000`
baud with frame `laser`.

External LiDAR drivers live in `/home/rudra/ros2_ws`, then RUDRAv4 is sourced
on top:

```bash
source /opt/ros/lyrical/setup.bash
source /home/rudra/ros2_ws/install/setup.bash
source /home/rudra/Projects/RUDRAv4/install/setup.bash
```

Find the LiDAR USB port:

```bash
bash scripts/list_serial.sh
```

Prefer the stable `/dev/serial/by-id/...` path for the LiDAR adapter. If only
`/dev/ttyUSB0` appears, use that until a udev rule or by-id path is available.

Run the LiDAR driver:

```bash
bash scripts/run_lidar.sh /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

Open RViz to see the scan and obstacle guard marker:

```bash
bash scripts/view_lidar.sh
```

RViz displays `/scan` as green points with `base_link` as the fixed frame and a
static transform to the LiDAR `laser` frame. When a bridge node is running,
`/rudra/obstacle_guard_marker` shows the closest front-cone
obstacle used by the guard: green is clear, orange is slowing, red is blocked,
and gray means the guard is disabled.

The default RUDRAv4 LiDAR launch uses `ydlidar_ros2_driver` with
`src/rudra_base_bridge/config/ydlidar_g2b.yaml` and the LiDAR adapter's
`/dev/serial/by-id/...` path, `128000` baud, `sample_rate: 5`,
`frequency: 10.0`, `intensity: true`, and scan frame `laser`.

The spinning LiDAR and the obstacle guard are separate. Keep the driver
publishing `/scan` for SLAM/Nav, and enable or disable only the low-level drive
filter when needed:

```bash
ros2 topic pub --once /rudra/obstacle_guard_enable std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /rudra/obstacle_guard_enable std_msgs/msg/Bool "{data: true}"
ros2 topic echo /rudra/obstacle_guard
```

From the PS2 controller, press `START` to toggle the obstacle guard latch in
the Uno firmware. MODE would be a natural label, but the vendored PS2X library
does not expose MODE as a normal readable button. The latched seventh `guard`
field does not stop `/scan`; it only changes whether the guard filters forward
motor commands.

Tune these parameters in
`src/rudra_base_bridge/config/rudra_v4_hardware.yaml` for both
`ps2_uno_to_teensy` and `cmd_vel_to_teensy`:

```yaml
obstacle_avoidance_enabled: true
obstacle_enable_topic: "/rudra/obstacle_guard_enable"
scan_topic: "/scan"
obstacle_front_angle_deg: 70.0
obstacle_stop_distance_m: 0.45
obstacle_slow_distance_m: 1.00
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
J,select,ry,rx,ly,lx,guard
```

The ROS2 bridge also accepts the old six-field packet during transition, but
the PS2 guard latch only works after reflashing the Uno with the updated
firmware.

NUC to Teensy:

```text
D,left,right,enable
```

`left` and `right` use the Sabertooth command range `-127..127`.
