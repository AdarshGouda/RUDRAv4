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
sudo tee /etc/udev/rules.d/99-dcdc-usb.rules >/dev/null <<'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="04d8", ATTR{idProduct}=="d003", MODE="0660", GROUP="plugdev", TAG+="uaccess"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger

cat <<'EOF'

DCDC-USB utility and udev rule installed.

Unplug and reconnect the DCDC-USB USB cable, then test:

  dcdc-usb -a

EOF
