"""Serial helpers for RUDRA base bridge."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Optional

import serial


@dataclass
class SerialLineResult:
    line: Optional[str]
    error: Optional[str] = None


class LineSerial:
    """Small line-oriented pyserial wrapper with reconnect support."""

    def __init__(self, port: str, baud: int, timeout: float = 0.02) -> None:
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self._ser: Optional[serial.Serial] = None
        self._last_open_attempt = 0.0

    @property
    def connected(self) -> bool:
        return bool(self._ser and self._ser.is_open)

    def _open(self) -> bool:
        if self.connected:
            return True

        now = time.monotonic()
        if now - self._last_open_attempt < 1.0:
            return False
        self._last_open_attempt = now

        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            time.sleep(0.2)
            return True
        except serial.SerialException:
            self._ser = None
            return False

    def close(self) -> None:
        if self._ser:
            try:
                self._ser.close()
            except serial.SerialException:
                pass
        self._ser = None

    def readline(self) -> SerialLineResult:
        if not self._open() or self._ser is None:
            return SerialLineResult(None, 'not_connected')
        try:
            raw = self._ser.readline()
            if not raw:
                return SerialLineResult(None, None)
            return SerialLineResult(raw.decode(errors='ignore').strip(), None)
        except serial.SerialException as exc:
            self.close()
            return SerialLineResult(None, str(exc))

    def write_line(self, line: str) -> bool:
        if not self._open() or self._ser is None:
            return False
        try:
            if not line.endswith('\n'):
                line += '\n'
            self._ser.write(line.encode('ascii', errors='ignore'))
            return True
        except serial.SerialException:
            self.close()
            return False

    def drain_available(self, max_lines: int = 10) -> list[str]:
        lines: list[str] = []
        if not self._open() or self._ser is None:
            return lines
        try:
            count = 0
            while self._ser.in_waiting and count < max_lines:
                raw = self._ser.readline()
                if raw:
                    lines.append(raw.decode(errors='ignore').strip())
                count += 1
        except serial.SerialException:
            self.close()
        return lines


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def clamp_int(value: int, low: int, high: int) -> int:
    return int(max(low, min(high, value)))


def map_range(value: float, in_min: float, in_max: float, out_min: float, out_max: float) -> float:
    if in_max == in_min:
        return out_min
    ratio = (value - in_min) / (in_max - in_min)
    return out_min + ratio * (out_max - out_min)


def apply_deadband(value: int, deadband: int) -> int:
    return 0 if abs(value) <= deadband else value
