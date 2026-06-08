#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DCDC_SRC="${REPO_ROOT}/reference/DCDC/dcdc-usb"

if ! pkg-config --exists libusb; then
  sudo apt update
  sudo apt install -y libusb-dev || sudo apt install -y libusb-0.1-4-dev
fi

make -C "${DCDC_SRC}" clean
make -C "${DCDC_SRC}"
sudo make -C "${DCDC_SRC}" install

cat <<'EOF'

DCDC-USB utility installed.

If `dcdc-usb -a` only works with sudo, add a udev rule like:

  SUBSYSTEM=="usb", ATTR{idVendor}=="04d8", ATTR{idProduct}=="d003", MODE="0660", GROUP="plugdev", TAG+="uaccess"

Then reload rules and reconnect the DCDC-USB:

  sudo udevadm control --reload-rules
  sudo udevadm trigger

EOF

