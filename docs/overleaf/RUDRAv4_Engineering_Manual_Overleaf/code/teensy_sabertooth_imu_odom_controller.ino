/*
  RUDRAv4 Teensy Sabertooth firmware with IMU + encoder telemetry.

  Preserves the current plain-serial command input:
    D,left,right,enable

  Adds plain-serial telemetry back to the NUC:
    IMU,ax,ay,az,gx,gy,gz
    ODOM,x,y,theta,linear_x,angular_z,left_vel,right_vel

  This sketch intentionally keeps the existing open-loop Sabertooth command
  behavior. Encoder odometry and MPU6050 telemetry are added first so the ROS 2
  stack can bring up IMU and localization before deciding whether PID is needed.

  Required Arduino libraries:
    - Encoder
    - I2Cdev
    - MPU6050
    - Sabertooth
*/

#include <Sabertooth.h>
#include <Wire.h>

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

#include <I2Cdev.h>
#include <MPU6050.h>

Sabertooth STRight(128, Serial1);
Sabertooth STLeft(129, Serial1);

MPU6050 accelgyro(0x69);

Encoder EncR1(8, 7);
Encoder EncL1(2, 3);
Encoder EncL2(4, 5);
Encoder EncR2(32, 31);

const uint32_t CMD_BAUD = 115200;
const uint32_t SABER_BAUD = 9600;
const uint32_t CONTROL_PERIOD_MS = 20;
const uint32_t TELEMETRY_PERIOD_MS = 50;
const uint32_t COMMAND_TIMEOUT_MS = 300;
const uint32_t HEARTBEAT_PERIOD_MS = 500;
const uint32_t HEARTBEAT_ON_MS = 100;

const int MAX_CMD = 127;
const int RAMP_STEP = 6;

const double RADIUS_M = 0.0675;
const double WHEELBASE_M = 0.29;
const double ENCODER_CPR = 1683.0;
const double kTwoPi = 6.28318530718;

const double ACCEL_SCALE = 1.0 / 16384.0;
const double GYRO_SCALE = 1.0 / 131.0;
const double G_TO_ACCEL = 9.81;
const double kDegToRad = 0.01745329252;

const bool FUSE_IMU_YAW = true;
const double FUSE_ALPHA = 0.5;

String lineBuffer;

int targetLeft = 0;
int targetRight = 0;
int actualLeft = 0;
int actualRight = 0;
bool driveEnabled = false;

int16_t ax = 0, ay = 0, az = 0;
int16_t gx = 0, gy = 0, gz = 0;
double gyroBiasX = 0.0;
double gyroBiasY = 0.0;
double gyroBiasZ = 0.0;
double accBiasX = 0.0;
double accBiasY = 0.0;
double accBiasZ = 0.0;

long prevCountL1 = 0;
long prevCountL2 = 0;
long prevCountR1 = 0;
long prevCountR2 = 0;

double xPos = 0.0;
double yPos = 0.0;
double theta = 0.0;

uint32_t lastCommandMs = 0;
uint32_t lastControlMs = 0;
uint32_t lastTelemetryMs = 0;
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
  targetLeft = driveEnabled ? left : 0;
  targetRight = driveEnabled ? right : 0;
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
      if (lineBuffer.length() < 96) {
        lineBuffer += c;
      } else {
        lineBuffer = "";
        Serial.println("ERR,line_too_long");
      }
    }
  }
}

void calibrateImuBiases() {
  if (!accelgyro.testConnection()) {
    Serial.println("ERR,imu_not_found");
    return;
  }

  for (int i = 0; i < 200; ++i) {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    accBiasX += ax;
    accBiasY += ay;
    accBiasZ += az;
    gyroBiasX += gx;
    gyroBiasY += gy;
    gyroBiasZ += gz;
    delay(2);
  }

  accBiasX /= 200.0;
  accBiasY /= 200.0;
  accBiasZ /= 200.0;
  gyroBiasX /= 200.0;
  gyroBiasY /= 200.0;
  gyroBiasZ /= 200.0;
}

double encoderSpeed(long deltaCounts, double dtSec) {
  if (dtSec <= 0.0) return 0.0;
  return ((double)deltaCounts / ENCODER_CPR) * kTwoPi * RADIUS_M / dtSec;
}

void publishTelemetry(double dtSec) {
  long countL1 = EncL1.read();
  long countL2 = EncL2.read();
  long countR1 = EncR1.read();
  long countR2 = EncR2.read();

  long deltaL1 = countL1 - prevCountL1;
  long deltaL2 = countL2 - prevCountL2;
  long deltaR1 = countR1 - prevCountR1;
  long deltaR2 = countR2 - prevCountR2;

  prevCountL1 = countL1;
  prevCountL2 = countL2;
  prevCountR1 = countR1;
  prevCountR2 = countR2;

  double speedLeft1 = encoderSpeed(deltaL1, dtSec);
  double speedLeft2 = encoderSpeed(deltaL2, dtSec);
  double speedRight1 = encoderSpeed(deltaR1, dtSec);
  double speedRight2 = encoderSpeed(deltaR2, dtSec);

  double speedLeftAvg = (speedLeft1 + speedLeft2) / 2.0;
  double speedRightAvg = (speedRight1 + speedRight2) / 2.0;

  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  double accX = (ax - accBiasX) * ACCEL_SCALE * G_TO_ACCEL;
  double accY = (ay - accBiasY) * ACCEL_SCALE * G_TO_ACCEL;
  double accZ = (az - accBiasZ) * ACCEL_SCALE * G_TO_ACCEL;

  double gyroX = (gx - gyroBiasX) * GYRO_SCALE * kDegToRad;
  double gyroY = (gy - gyroBiasY) * GYRO_SCALE * kDegToRad;
  double gyroZ = (gz - gyroBiasZ) * GYRO_SCALE * kDegToRad;

  double linearX = (speedLeftAvg + speedRightAvg) / 2.0;
  double angularZEnc = (speedRightAvg - speedLeftAvg) / WHEELBASE_M;
  double dthOdom = angularZEnc * dtSec;
  double dth = dthOdom;
  if (FUSE_IMU_YAW) {
    dth = FUSE_ALPHA * dthOdom + (1.0 - FUSE_ALPHA) * gyroZ * dtSec;
  }

  theta += dth;
  if (theta >= kTwoPi) theta -= kTwoPi;
  if (theta <= -kTwoPi) theta += kTwoPi;

  double dxy = linearX * dtSec;
  xPos += cos(theta) * dxy;
  yPos += sin(theta) * dxy;

  Serial.print("IMU,");
  Serial.print(accX, 6);
  Serial.print(",");
  Serial.print(accY, 6);
  Serial.print(",");
  Serial.print(accZ, 6);
  Serial.print(",");
  Serial.print(gyroX, 6);
  Serial.print(",");
  Serial.print(gyroY, 6);
  Serial.print(",");
  Serial.println(gyroZ, 6);

  Serial.print("ODOM,");
  Serial.print(xPos, 6);
  Serial.print(",");
  Serial.print(yPos, 6);
  Serial.print(",");
  Serial.print(theta, 6);
  Serial.print(",");
  Serial.print(linearX, 6);
  Serial.print(",");
  Serial.print(angularZEnc, 6);
  Serial.print(",");
  Serial.print(speedLeftAvg, 6);
  Serial.print(",");
  Serial.println(speedRightAvg, 6);
}

void setup() {
  pinMode(HEARTBEAT_LED_PIN, OUTPUT);
  digitalWrite(HEARTBEAT_LED_PIN, LOW);

  Serial.begin(CMD_BAUD);
  Serial1.begin(SABER_BAUD);
  Wire.begin();

  delay(2000);
  Sabertooth::autobaud(Serial1);

  STRight.setTimeout(300);
  STLeft.setTimeout(300);
  accelgyro.initialize();
  accelgyro.setI2CBypassEnabled(true);
  calibrateImuBiases();

  stopBaseImmediate();
  Serial.println("BOOT,RUDRA_TEENSY_IMU_ODOM_READY");
}

void loop() {
  uint32_t now = millis();

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

  if ((now - lastTelemetryMs) >= TELEMETRY_PERIOD_MS) {
    double dtSec = (lastTelemetryMs == 0)
      ? (TELEMETRY_PERIOD_MS / 1000.0)
      : ((now - lastTelemetryMs) / 1000.0);
    lastTelemetryMs = now;
    publishTelemetry(dtSec);
  }
}
