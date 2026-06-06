/*
  RUDRAv4 Uno PS2 plain serial firmware.

  Purpose:
    Read a PlayStation 2 controller through the PS2X library and forward the
    stick values over USB serial in a compact line-oriented format.

  Verified wiring from RUDRAv3:
    CLK = 9
    CMD = 7
    ATT = 6
    DAT = 8

  Serial output:
    J,select,ry,rx,ly,lx

    - `select` is a 0/1 manual-mode flag toggled by the SELECT button
    - `ry`, `rx`, `ly`, and `lx` are the raw stick readings from PS2X

  Notes for future maintainers:
    - The controller must be connected before power-up or reset.
    - The PS2X library is vendored alongside this sketch so the build does not
      depend on a global Arduino library install.
    - If the controller stops reading correctly, confirm the pin mapping first.
*/

#include "PS2X_lib.h"

PS2X ps2x;

const uint32_t SERIAL_BAUD = 115200;
const uint16_t LOOP_DELAY_MS = 50;  // 20 Hz

bool manual_mode = false;
bool last_select = false;
byte vibrate = 0;
int error = 0;

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  // Initialize the PS2 controller interface using the wiring validated on RUDRAv3.
  // We keep pressures and rumble enabled because that was the known working setup.
  error = ps2x.config_gamepad(9, 7, 6, 8, true, true);

  // Startup status is intentionally minimal so operators can quickly spot
  // whether the controller handshake succeeded.
  Serial.print("BOOT,PS2_ERROR,");
  Serial.println(error);
}

void loop() {
  // Poll the controller once per loop. The library updates the internal button
  // and stick state, which we then expose over serial below.
  ps2x.read_gamepad(false, vibrate);

  // SELECT is treated as a toggle for manual mode rather than a momentary input.
  // We only flip state on the rising edge to avoid repeated toggles while held.
  bool select_now = ps2x.Button(PSB_SELECT);
  if (select_now && !last_select) {
    manual_mode = !manual_mode;
  }
  last_select = select_now;

  // Emit one compact line that the bridge software can parse reliably.
  Serial.print("J,");
  Serial.print(manual_mode ? 1 : 0);
  Serial.print(",");
  Serial.print(ps2x.Analog(PSS_RY));
  Serial.print(",");
  Serial.print(ps2x.Analog(PSS_RX));
  Serial.print(",");
  Serial.print(ps2x.Analog(PSS_LY));
  Serial.print(",");
  Serial.println(ps2x.Analog(PSS_LX));

  delay(LOOP_DELAY_MS);
}
