# DCDC-USB NUC Power Monitor

RUDRAv4 can power the NUC from a dedicated 4S LiPo through the Mini-Box
DCDC-USB buck-boost converter, while keeping ROS aware of that power rail.

The first integration is intentionally monitor-first:

- the DCDC-USB jumper or vendor configuration controls the output voltage
- ROS reads the DCDC-USB over USB through the Mini-Box Linux utility
- ROS publishes battery state and diagnostics
- automatic NUC shutdown is available, but disabled by default

## Electrical Plan

```text
4S LiPo, 14.8 V nominal / 16.8 V full
  -> DCDC-USB input
  -> regulated DCDC-USB output
  -> NUC DC input

DCDC-USB mini USB
  -> NUC USB
  -> ROS dcdc_usb_monitor node
```

The Mini-Box DCDC-USB product page
<https://www.mini-box.com/DCDC-USB> lists the converter as a 100 W buck-boost
unit with 6-34 V input, programmable 5-24 V output, 12 V default output, and USB
configuration/status support.

Before plugging the NUC into the DCDC output, verify the NUC's accepted input
range from the NUC label or datasheet. Many NUC power bricks are 19 V, while
some NUC models tolerate a wider 12-19 V range. Do not assume the DCDC-USB
default 12 V output is correct for your NUC.

## Install The DCDC-USB Linux Utility

The ROS node wraps the vendor `dcdc-usb -a` command so you can debug the same
path outside ROS.

```bash
cd /home/rudra/Projects/RUDRAv4
bash scripts/install_dcdc_usb_tool.sh
```

The installer also writes `/etc/udev/rules.d/99-dcdc-usb.rules`. Unplug and
reconnect the DCDC-USB USB cable after the installer completes so Linux applies
the new device permissions.

Check the board manually:

```bash
dcdc-usb -a
```

Expected fields include:

```text
input voltage: 15.42
output voltage: 12.06
mode: 0 (dumb)
output enable: On
```

If `dcdc-usb -a` reports `Cannot claim interface 0`, reconnect the USB cable
first. If it still fails, run `sudo dcdc-usb -a` once to distinguish a
permissions issue from a kernel-driver claim issue.

## Run The ROS Monitor

Standalone:

```bash
source /opt/ros/lyrical/setup.bash
source install/setup.bash
ros2 launch rudra_base_bridge dcdc_usb_monitor.launch.py
```

With PS2, LiDAR, IMU/wheel odom, and localization:

```bash
ros2 launch rudra_base_bridge rudra_ps2_lidar_localization.launch.py enable_dcdc_monitor:=true
```

Useful monitors:

```bash
ros2 topic echo /rudra/nuc_battery
ros2 topic echo /diagnostics
```

The node publishes:

- `/rudra/nuc_battery` as `sensor_msgs/BatteryState`
- `/diagnostics` as `diagnostic_msgs/DiagnosticArray`

## Configuration

Defaults live in:

```text
src/rudra_base_bridge/config/rudra_v4_hardware.yaml
```

Important parameters:

```yaml
dcdc_usb_monitor:
  ros__parameters:
    dcdc_usb_command: "dcdc-usb -a"
    cell_count: 4
    full_voltage: 16.8
    warning_voltage: 14.4
    critical_voltage: 13.2
    shutdown_voltage: 12.8
    expected_output_voltage: 12.0
    output_voltage_tolerance: 1.0
    shutdown_enabled: false
```

For a 4S LiPo:

- 16.8 V is full charge
- 14.8 V is nominal
- 14.4 V is a conservative warning point
- 13.2 V is 3.3 V/cell and treated as critical
- 12.8 V is reserved for optional shutdown testing, not enabled by default

The monitor estimates state-of-charge from pack voltage linearly. That is only
a rough dashboard hint, because LiPo voltage sag depends on load and battery
condition.

## Optional Graceful Shutdown

Leave shutdown disabled until the DCDC readings match a multimeter and the NUC
load has been tested.

After validation, enable:

```yaml
shutdown_enabled: true
shutdown_voltage: 12.8
shutdown_command: "sudo shutdown -h now"
```

The node will run the shutdown command only once when input voltage reaches the
shutdown threshold.
