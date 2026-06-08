"""ROS monitor for the Mini-Box DCDC-USB powering the NUC."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math
import re
import shlex
import subprocess
import time
from typing import Optional

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node

from rudra_base_bridge.serial_utils import clamp
from sensor_msgs.msg import BatteryState


_KEY_VALUE_RE = re.compile(r'^\s*([^:]+):\s*(.*?)\s*$')
_MODE_RE = re.compile(r'^(?P<number>\d+)(?:\s*\((?P<name>[^)]+)\))?')

DIAG_OK = b'\x00'
DIAG_WARN = b'\x01'
DIAG_ERROR = b'\x02'
DIAG_STALE = b'\x03'


@dataclass(frozen=True)
class DcdcUsbStatus:
    """Parsed status values from `dcdc-usb -a`."""

    input_voltage: Optional[float] = None
    ignition_voltage: Optional[float] = None
    output_voltage: Optional[float] = None
    programmed_output_voltage: Optional[float] = None
    mode_number: Optional[int] = None
    mode_name: str = ''
    state: Optional[int] = None
    power_switch_on: Optional[bool] = None
    output_enabled: Optional[bool] = None
    aux_vin_enabled: Optional[bool] = None
    version: str = ''
    raw_values: dict[str, str] = field(default_factory=dict)


def _parse_optional_float(value: Optional[str]) -> Optional[float]:
    if value is None:
        return None
    try:
        return float(value.strip())
    except ValueError:
        return None


def _parse_optional_int(value: Optional[str]) -> Optional[int]:
    if value is None:
        return None
    try:
        return int(value.strip())
    except ValueError:
        return None


def _parse_on_off(value: Optional[str]) -> Optional[bool]:
    if value is None:
        return None
    normalized = value.strip().lower()
    if normalized == 'on':
        return True
    if normalized == 'off':
        return False
    return None


def parse_dcdc_usb_output(output: str) -> DcdcUsbStatus:
    """Parse human-readable output from the Mini-Box dcdc-usb utility."""
    raw_values: dict[str, str] = {}

    for line in output.splitlines():
        line = line.strip()
        if not line:
            continue

        match = _KEY_VALUE_RE.match(line)
        if match:
            key = match.group(1).strip().lower()
            if key == 'output voltage' and key in raw_values:
                raw_values['programmed output voltage'] = match.group(2).strip()
            else:
                raw_values[key] = match.group(2).strip()
            continue

        # The Linux parser prints this one without a colon.
        prefix = 'aux vin enable '
        if line.lower().startswith(prefix):
            raw_values['aux vin enable'] = line[len(prefix):].strip()

    mode_number = None
    mode_name = ''
    mode_match = _MODE_RE.match(raw_values.get('mode', ''))
    if mode_match:
        mode_number = int(mode_match.group('number'))
        mode_name = mode_match.group('name') or ''

    return DcdcUsbStatus(
        input_voltage=_parse_optional_float(raw_values.get('input voltage')),
        ignition_voltage=_parse_optional_float(raw_values.get('ignition voltage')),
        output_voltage=_parse_optional_float(raw_values.get('output voltage')),
        programmed_output_voltage=_parse_optional_float(
            raw_values.get('programmed output voltage')
        ),
        mode_number=mode_number,
        mode_name=mode_name,
        state=_parse_optional_int(raw_values.get('state')),
        power_switch_on=_parse_on_off(raw_values.get('power switch')),
        output_enabled=_parse_on_off(raw_values.get('output enable')),
        aux_vin_enabled=_parse_on_off(raw_values.get('aux vin enable')),
        version=raw_values.get('version', ''),
        raw_values=raw_values,
    )


class DcdcUsbCommandReader:
    """Read DCDC-USB status using the vendor command-line utility."""

    def __init__(self, command: str, timeout_sec: float) -> None:
        self.command = command
        self.timeout_sec = timeout_sec

    def read(self) -> tuple[Optional[DcdcUsbStatus], Optional[str]]:
        args = shlex.split(self.command)
        if not args:
            return None, 'dcdc_usb_command is empty'

        try:
            result = subprocess.run(
                args,
                capture_output=True,
                check=False,
                text=True,
                timeout=self.timeout_sec,
            )
        except FileNotFoundError:
            return None, f'command not found: {args[0]}'
        except subprocess.TimeoutExpired:
            return None, f'command timed out after {self.timeout_sec:.1f}s'
        except OSError as exc:
            return None, str(exc)

        output = '\n'.join(part for part in (result.stdout, result.stderr) if part)
        if result.returncode != 0:
            return None, output.strip() or f'command exited {result.returncode}'

        status = parse_dcdc_usb_output(output)
        if status.input_voltage is None and status.output_voltage is None:
            return None, 'DCDC-USB output did not contain voltage fields'
        return status, None


class DcdcUsbMonitorNode(Node):
    """Publish ROS battery and diagnostic state for the NUC power rail."""

    def __init__(self) -> None:
        super().__init__('dcdc_usb_monitor')

        self.declare_parameter('dcdc_usb_command', 'dcdc-usb -a')
        self.declare_parameter('poll_period_sec', 1.0)
        self.declare_parameter('command_timeout_sec', 2.0)
        self.declare_parameter('battery_topic', '/rudra/nuc_battery')
        self.declare_parameter('diagnostics_topic', '/diagnostics')
        self.declare_parameter('battery_frame_id', 'nuc_power')
        self.declare_parameter('diagnostics_name', 'rudra_nuc_dcdc')
        self.declare_parameter('cell_count', 4)
        self.declare_parameter('full_voltage', 16.8)
        self.declare_parameter('empty_voltage', 13.2)
        self.declare_parameter('warning_voltage', 14.4)
        self.declare_parameter('critical_voltage', 13.2)
        self.declare_parameter('shutdown_voltage', 12.8)
        self.declare_parameter('expected_output_min_voltage', 19.0)
        self.declare_parameter('expected_output_max_voltage', 20.5)
        self.declare_parameter('input_sag_window_sec', 10.0)
        self.declare_parameter('input_sag_warning_voltage', 0.8)
        self.declare_parameter('shutdown_enabled', False)
        self.declare_parameter('shutdown_hold_sec', 5.0)
        self.declare_parameter('shutdown_command', 'sudo shutdown -h now')

        command = str(self.get_parameter('dcdc_usb_command').value)
        timeout_sec = float(self.get_parameter('command_timeout_sec').value)
        self.reader = DcdcUsbCommandReader(command, timeout_sec)

        self.battery_frame_id = str(self.get_parameter('battery_frame_id').value)
        self.diagnostics_name = str(self.get_parameter('diagnostics_name').value)
        self.cell_count = int(self.get_parameter('cell_count').value)
        self.full_voltage = float(self.get_parameter('full_voltage').value)
        self.empty_voltage = float(self.get_parameter('empty_voltage').value)
        self.warning_voltage = float(self.get_parameter('warning_voltage').value)
        self.critical_voltage = float(self.get_parameter('critical_voltage').value)
        self.shutdown_voltage = float(self.get_parameter('shutdown_voltage').value)
        self.expected_output_min_voltage = float(
            self.get_parameter('expected_output_min_voltage').value
        )
        self.expected_output_max_voltage = float(
            self.get_parameter('expected_output_max_voltage').value
        )
        self.input_sag_window_sec = float(
            self.get_parameter('input_sag_window_sec').value
        )
        self.input_sag_warning_voltage = float(
            self.get_parameter('input_sag_warning_voltage').value
        )
        self.shutdown_enabled = bool(self.get_parameter('shutdown_enabled').value)
        self.shutdown_hold_sec = float(self.get_parameter('shutdown_hold_sec').value)
        self.shutdown_command = str(self.get_parameter('shutdown_command').value)

        self.battery_pub = self.create_publisher(
            BatteryState,
            str(self.get_parameter('battery_topic').value),
            10,
        )
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter('diagnostics_topic').value),
            10,
        )

        self.shutdown_triggered = False
        self._shutdown_low_since: Optional[float] = None
        self._input_voltage_history: deque[tuple[float, float]] = deque()
        self._last_error_log_sec = 0.0
        poll_period_sec = float(self.get_parameter('poll_period_sec').value)
        self.timer = self.create_timer(poll_period_sec, self._poll)

        self.get_logger().info(f'Monitoring DCDC-USB with: {command}')

    def _poll(self) -> None:
        status, error = self.reader.read()
        if status is None:
            self._publish_stale(error or 'DCDC-USB status unavailable')
            self._log_error_rate_limited(error or 'DCDC-USB status unavailable')
            return

        self._record_input_voltage(status)
        self.battery_pub.publish(self._make_battery_msg(status))
        self.diagnostics_pub.publish(self._make_diagnostics_msg(status))
        self._maybe_shutdown(status)

    def _make_battery_msg(self, status: DcdcUsbStatus) -> BatteryState:
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.battery_frame_id
        msg.voltage = self._nan_if_missing(status.input_voltage)
        msg.current = math.nan
        msg.charge = math.nan
        msg.capacity = math.nan
        msg.design_capacity = math.nan
        msg.percentage = self._estimate_percentage(status.input_voltage)
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = self._battery_health(status)
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        msg.present = status.input_voltage is not None
        if self.cell_count > 0 and status.input_voltage is not None:
            msg.cell_voltage = [status.input_voltage / self.cell_count] * self.cell_count
        msg.location = 'nuc_dcdc_input'
        return msg

    def _make_diagnostics_msg(self, status: DcdcUsbStatus) -> DiagnosticArray:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        diagnostic = DiagnosticStatus()
        diagnostic.name = self.diagnostics_name
        diagnostic.hardware_id = 'Mini-Box DCDC-USB 04d8:d003'
        level, message = self._diagnostic_state(status)
        diagnostic.level = level  # type: ignore[assignment]
        diagnostic.message = message
        diagnostic.values = self._diagnostic_values(status)
        array.status.append(diagnostic)
        return array

    def _publish_stale(self, message: str) -> None:
        self.battery_pub.publish(self._make_missing_battery_msg())

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        diagnostic = DiagnosticStatus()
        diagnostic.name = self.diagnostics_name
        diagnostic.hardware_id = 'Mini-Box DCDC-USB 04d8:d003'
        diagnostic.level = DIAG_STALE  # type: ignore[assignment]
        diagnostic.message = message
        array.status.append(diagnostic)
        self.diagnostics_pub.publish(array)

    def _make_missing_battery_msg(self) -> BatteryState:
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.battery_frame_id
        msg.voltage = math.nan
        msg.current = math.nan
        msg.charge = math.nan
        msg.capacity = math.nan
        msg.design_capacity = math.nan
        msg.percentage = math.nan
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        msg.present = False
        msg.location = 'nuc_dcdc_input'
        return msg

    def _diagnostic_state(self, status: DcdcUsbStatus) -> tuple[bytes, str]:
        input_voltage = status.input_voltage
        output_voltage = status.output_voltage

        if input_voltage is None:
            return DIAG_STALE, 'missing input voltage'
        if input_voltage <= self.critical_voltage:
            return DIAG_ERROR, 'NUC LiPo input voltage critical'
        if input_voltage <= self.warning_voltage:
            return DIAG_WARN, 'NUC LiPo input voltage low'

        if output_voltage is not None:
            if (
                status.output_enabled is True
                and output_voltage < self.expected_output_min_voltage
            ):
                return DIAG_ERROR, 'DCDC output below expected range'
            if (
                status.output_enabled is True
                and output_voltage > self.expected_output_max_voltage
            ):
                return DIAG_WARN, 'DCDC output above expected range'
            configured = status.programmed_output_voltage
            if configured is not None and not self._output_in_expected_range(configured):
                return DIAG_WARN, 'DCDC programmed output differs from expected range'

        if status.output_enabled is False:
            return DIAG_WARN, 'DCDC output is disabled'

        input_sag = self._input_voltage_sag()
        if (
            input_sag is not None
            and input_sag >= self.input_sag_warning_voltage
        ):
            return DIAG_WARN, 'NUC input voltage sag suggests high load or weak pack'

        return DIAG_OK, 'NUC DCDC power nominal'

    def _diagnostic_values(self, status: DcdcUsbStatus) -> list[KeyValue]:
        values = [
            self._kv('input_voltage_v', status.input_voltage),
            self._kv('output_voltage_v', status.output_voltage),
            self._kv('programmed_output_voltage_v', status.programmed_output_voltage),
            self._kv('ignition_voltage_v', status.ignition_voltage),
            self._kv('estimated_soc', self._estimate_percentage(status.input_voltage)),
            self._kv('cell_count', self.cell_count),
            self._kv('average_cell_voltage_v', self._average_cell_voltage(status)),
            self._kv('input_voltage_sag_v', self._input_voltage_sag()),
            self._kv('input_sag_window_sec', self.input_sag_window_sec),
            self._kv('expected_output_min_voltage_v', self.expected_output_min_voltage),
            self._kv('expected_output_max_voltage_v', self.expected_output_max_voltage),
            self._kv('mode_number', status.mode_number),
            self._kv('mode_name', status.mode_name),
            self._kv('state', status.state),
            self._kv('power_switch_on', status.power_switch_on),
            self._kv('output_enabled', status.output_enabled),
            self._kv('aux_vin_enabled', status.aux_vin_enabled),
            self._kv('version', status.version),
        ]
        return values

    def _maybe_shutdown(self, status: DcdcUsbStatus) -> None:
        if not self.shutdown_enabled or self.shutdown_triggered:
            return
        if status.input_voltage is None or status.input_voltage > self.shutdown_voltage:
            self._shutdown_low_since = None
            return

        now = time.monotonic()
        if self._shutdown_low_since is None:
            self._shutdown_low_since = now
            return
        if now - self._shutdown_low_since < self.shutdown_hold_sec:
            return

        args = shlex.split(self.shutdown_command)
        if not args:
            self.get_logger().error('shutdown_command is empty; cannot shut down')
            return

        self.shutdown_triggered = True
        self.get_logger().error(
            'NUC LiPo voltage reached shutdown threshold; running shutdown command'
        )
        try:
            subprocess.Popen(args)
        except OSError as exc:
            self.get_logger().error(f'Failed to run shutdown command: {exc}')

    def _record_input_voltage(self, status: DcdcUsbStatus) -> None:
        if status.input_voltage is None:
            return

        now = time.monotonic()
        self._input_voltage_history.append((now, status.input_voltage))
        cutoff = now - self.input_sag_window_sec
        while self._input_voltage_history and self._input_voltage_history[0][0] < cutoff:
            self._input_voltage_history.popleft()

    def _input_voltage_sag(self) -> Optional[float]:
        if len(self._input_voltage_history) < 2:
            return None

        voltages = [sample[1] for sample in self._input_voltage_history]
        sag = max(voltages) - voltages[-1]
        return max(0.0, sag)

    def _output_in_expected_range(self, voltage: float) -> bool:
        return (
            self.expected_output_min_voltage
            <= voltage
            <= self.expected_output_max_voltage
        )

    def _estimate_percentage(self, input_voltage: Optional[float]) -> float:
        if input_voltage is None:
            return math.nan
        if self.full_voltage <= self.empty_voltage:
            return math.nan
        percentage = (input_voltage - self.empty_voltage) / (
            self.full_voltage - self.empty_voltage
        )
        return clamp(percentage, 0.0, 1.0)

    def _average_cell_voltage(self, status: DcdcUsbStatus) -> Optional[float]:
        if self.cell_count <= 0 or status.input_voltage is None:
            return None
        return status.input_voltage / self.cell_count

    def _battery_health(self, status: DcdcUsbStatus) -> int:
        if status.input_voltage is None:
            return BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        if status.input_voltage <= self.critical_voltage:
            return BatteryState.POWER_SUPPLY_HEALTH_DEAD
        if status.input_voltage <= self.warning_voltage:
            return BatteryState.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE
        return BatteryState.POWER_SUPPLY_HEALTH_GOOD

    def _log_error_rate_limited(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_error_log_sec < 10.0:
            return
        self._last_error_log_sec = now
        self.get_logger().warning(message)

    @staticmethod
    def _nan_if_missing(value: Optional[float]) -> float:
        return math.nan if value is None else value

    @staticmethod
    def _kv(key: str, value: object) -> KeyValue:
        msg = KeyValue()
        msg.key = key
        msg.value = '' if value is None else str(value)
        return msg


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = DcdcUsbMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
