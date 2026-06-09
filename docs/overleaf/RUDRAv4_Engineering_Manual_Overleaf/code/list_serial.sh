#!/usr/bin/env bash
set -euo pipefail
echo "Serial by-id devices:"
ls -l /dev/serial/by-id/ || true
echo
echo "USB ACM/USB tty devices:"
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true
