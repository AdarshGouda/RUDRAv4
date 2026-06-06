/*
  RUDRAv4 Uno PS2 plain serial firmware.

  This keeps the RUDRAv3 PS2 wiring:
    CLK = 9
    CMD = 7
    ATT = 6
    DAT = 8

  Output to NUC over USB serial:
    J,select,ry,rx,ly,lx

  SELECT toggles manual enable.
*/

#include <PS2X_lib.h>

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

  // RUDRAv3 pin order: CLK, CMD, ATT, DAT, pressures, rumble
  error = ps2x.config_gamepad(9, 7, 6, 8, true, true);

  Serial.print("BOOT,PS2_ERROR,");
  Serial.println(error);
}

void loop() {
  ps2x.read_gamepad(false, vibrate);

  bool select_now = ps2x.Button(PSB_SELECT);
  if (select_now && !last_select) {
    manual_mode = !manual_mode;
  }
  last_select = select_now;

  int ry = ps2x.Analog(PSS_RY);
  int rx = ps2x.Analog(PSS_RX);
  int ly = ps2x.Analog(PSS_LY);
  int lx = ps2x.Analog(PSS_LX);

  Serial.print("J,");
  Serial.print(manual_mode ? 1 : 0);
  Serial.print(",");
  Serial.print(ry);
  Serial.print(",");
  Serial.print(rx);
  Serial.print(",");
  Serial.print(ly);
  Serial.print(",");
  Serial.println(lx);

  delay(LOOP_DELAY_MS);
}
