"""List serial ports useful for RUDRA bringup."""

from __future__ import annotations

from serial.tools import list_ports


def main() -> None:
    ports = list(list_ports.comports())
    if not ports:
        print('No serial ports found.')
        return
    for port in ports:
        print(f'{port.device}\t{port.description}\t{port.hwid}')


if __name__ == '__main__':
    main()
