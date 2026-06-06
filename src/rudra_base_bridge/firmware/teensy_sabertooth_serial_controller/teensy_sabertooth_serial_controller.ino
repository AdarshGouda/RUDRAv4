/*
  RUDRAv4 Teensy Sabertooth plain serial firmware.

  RUDRAv3 hardware preserved:
    Right Sabertooth address = 128
    Left Sabertooth address  = 129
    Sabertooth packet serial = Serial1 / TX1, 9600 baud

  Input from NUC over USB serial:
    D,left,right,enable

  left/right range:
    -127..127
*/

#include <Sabertooth.h>

Sabertooth STRight(128, Serial1);
Sabertooth STLeft(129, Serial1);

const uint32_t CMD_BAUD = 115200;
const uint32_t SABER_BAUD = 9600;
const uint32_t CONTROL_PERIOD_MS = 20;
const uint32_t COMMAND_TIMEOUT_MS = 300;
const uint32_t HEARTBEAT_PERIOD_MS = 500;
const uint32_t HEARTBEAT_ON_MS = 100;

const int MAX_CMD = 127;
const int RAMP_STEP = 6;

String lineBuffer;

int targetLeft = 0;
int targetRight = 0;
int actualLeft = 0;
int actualRight = 0;
bool driveEnabled = false;

uint32_t lastCommandMs = 0;
uint32_t lastControlMs = 0;
uint32_t lastHeartbeatMs = 0;

const int HEARTBEAT_LED_PIN = LED_BUILTIN;

int clampInt(int value, int low, int high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

int rampToward(int current, int target, int step) {
  if (current < target) {
    current += step;
    if (current > target) current = target;
  } else if (current > target) {
    current -= step;
    if (current < target) current = target;
  }
  return current;
}

void stopBaseImmediate() {
  targetLeft = 0;
  targetRight = 0;
  actualLeft = 0;
  actualRight = 0;
  driveEnabled = false;

  STRight.motor(1, 0);
  STRight.motor(2, 0);
  STLeft.motor(1, 0);
  STLeft.motor(2, 0);
}

void applyBaseCommands(int leftCmd, int rightCmd) {
  // RUDRAv3 mapping: STRight drives right side, STLeft drives left side.
  STRight.motor(1, rightCmd);
  STRight.motor(2, rightCmd);
  STLeft.motor(1, leftCmd);
  STLeft.motor(2, leftCmd);
}

void parseCommand(String line) {
  line.trim();
  if (!line.startsWith("D,")) return;

  int c1 = line.indexOf(',');
  int c2 = line.indexOf(',', c1 + 1);
  int c3 = line.indexOf(',', c2 + 1);

  if (c1 < 0 || c2 < 0 || c3 < 0) {
    Serial.println("ERR,bad_format");
    return;
  }

  int left = line.substring(c1 + 1, c2).toInt();
  int right = line.substring(c2 + 1, c3).toInt();
  int enable = line.substring(c3 + 1).toInt();

  left = clampInt(left, -MAX_CMD, MAX_CMD);
  right = clampInt(right, -MAX_CMD, MAX_CMD);

  driveEnabled = (enable == 1);

  if (driveEnabled) {
    targetLeft = left;
    targetRight = right;
  } else {
    targetLeft = 0;
    targetRight = 0;
  }

  lastCommandMs = millis();

  Serial.print("ACK,");
  Serial.print(left);
  Serial.print(",");
  Serial.print(right);
  Serial.print(",");
  Serial.println(enable);
}

void readUsbCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineBuffer.length() > 0) {
        parseCommand(lineBuffer);
        lineBuffer = "";
      }
    } else {
      if (lineBuffer.length() < 80) {
        lineBuffer += c;
      } else {
        lineBuffer = "";
        Serial.println("ERR,line_too_long");
      }
    }
  }
}

void setup() {
  pinMode(HEARTBEAT_LED_PIN, OUTPUT);
  digitalWrite(HEARTBEAT_LED_PIN, LOW);

  Serial.begin(CMD_BAUD);
  Serial1.begin(SABER_BAUD);

  delay(2000);

  // One autobaud character on the shared Serial1 line is enough for both packet serial drivers.
  Sabertooth::autobaud(Serial1);

  STRight.setTimeout(300);
  STLeft.setTimeout(300);

  stopBaseImmediate();
  Serial.println("BOOT,RUDRA_TEENSY_SABERTOOTH_READY");
}

void loop() {
  uint32_t now = millis();

  // Blink the built-in LED twice per second: 100 ms on, 400 ms off.
  if ((now - lastHeartbeatMs) >= HEARTBEAT_PERIOD_MS) {
    lastHeartbeatMs = now;
    digitalWrite(HEARTBEAT_LED_PIN, HIGH);
  } else if ((now - lastHeartbeatMs) >= HEARTBEAT_ON_MS) {
    digitalWrite(HEARTBEAT_LED_PIN, LOW);
  }

  readUsbCommands();

  if ((now - lastCommandMs) > COMMAND_TIMEOUT_MS) {
    driveEnabled = false;
    targetLeft = 0;
    targetRight = 0;
  }

  if ((now - lastControlMs) >= CONTROL_PERIOD_MS) {
    lastControlMs = now;

    int step = driveEnabled ? RAMP_STEP : (RAMP_STEP * 2);
    actualLeft = rampToward(actualLeft, targetLeft, step);
    actualRight = rampToward(actualRight, targetRight, step);

    applyBaseCommands(actualLeft, actualRight);
  }
}
