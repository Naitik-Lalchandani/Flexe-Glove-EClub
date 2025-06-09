#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

const float MARKER = 1111.0;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  if (!mpu.begin()) {
    while (1);
  }
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float gx = g.gyro.x;
  float gy = g.gyro.y;
  float gz = g.gyro.z;

  Serial.write((uint8_t*)&gx, sizeof(float));
  Serial.write((uint8_t*)&gy, sizeof(float));
  Serial.write((uint8_t*)&gz, sizeof(float));
  Serial.write((uint8_t*)&MARKER, sizeof(float));  // send marker at end

  delay(5); // small delay to avoid flooding
}

