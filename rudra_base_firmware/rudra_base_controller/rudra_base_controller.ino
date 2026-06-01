/*
  RUDRA Base Controller - Teensy Firmware

  Current milestone:
    Send HEARTBEAT packet over USB serial.

  Future responsibilities:
    - Read MPU6050
    - Control 2x Sabertooth motor controllers
    - Receive CMD packets from NUC
    - Enforce watchdog safety
    - Send motor/IMU/odometry status to ROS 2 bridge

  Current serial output:
    HEARTBEAT,<millis>
*/

const int LED_PIN = 13;

const unsigned long HEARTBEAT_PERIOD_MS = 1000;
unsigned long last_heartbeat_ms = 0;

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);

  delay(2000);

  Serial.println("RUDRA_BASE_CONTROLLER_BOOT");
}

void loop()
{
  unsigned long now_ms = millis();

  if (now_ms - last_heartbeat_ms >= HEARTBEAT_PERIOD_MS)
  {
    last_heartbeat_ms = now_ms;

    Serial.print("HEARTBEAT,");
    Serial.println(now_ms);

    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}