#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Arduino.h>
#include <ESP32Servo.h>

Adafruit_MPU6050 mpu;
Servo pservo;
Servo yservo;
#define ALPHA 0.98

float pitch = 0.0; // rotation around X
float yaw   = 0.0; // rotation around Z (gyro only, will drift)

unsigned long lastTime = 0;

float gyroXoffset = 0.0;
float gyroYoffset = 0.0;
float gyroZoffset = 0.0;

int pos = 90;
int servoPos;

void calibrateGyro() {
  Serial.println("Calibrating — keep still...");
  const int samples = 500;
  double sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sumX += g.gyro.x;
    sumY += g.gyro.y;
    sumZ += g.gyro.z;
    delay(5);
  }
  gyroXoffset = sumX / samples;
  gyroYoffset = sumY / samples;
  gyroZoffset = sumZ / samples;
  Serial.println("Calibration done.");
}

void setup() {
  pservo.attach(0); 
  yservo.attach(1); 
  pinMode(5, INPUT);  
  Serial.begin(115200);
  while (!Serial) delay(10);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  calibrateGyro();

  // seed pitch from accelerometer, yaw starts at 0
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  pitch = atan2(a.acceleration.y, a.acceleration.z) * RAD_TO_DEG;
  yaw   = 0.0; // no absolute reference for yaw

  lastTime = micros();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  float gyroX = (g.gyro.x - gyroXoffset) * RAD_TO_DEG; // pitch rate
  float gyroZ = (g.gyro.z - gyroZoffset) * RAD_TO_DEG; // yaw rate

  // pitch — complementary filter with accel correction
  float accelPitch = atan2(a.acceleration.y, a.acceleration.z) * RAD_TO_DEG;
  pitch = ALPHA * (pitch + gyroX * dt) + (1.0 - ALPHA) * accelPitch;

  // yaw — gyro integration only, no accel correction possible
  yaw += gyroZ * dt;

  Serial.print("Pitch:"); Serial.print(pitch, 2);
  Serial.print(",Yaw:"); Serial.println(yaw, 2);
  
  servoPos = 90 + (int)pitch;
  servoPos = constrain(servoPos, 45, 135); // limit to ±20° from centre
  pservo.write(servoPos);
  servoPos = 90 + (int)yaw;
  servoPos = constrain(servoPos, 45, 135); // limit to ±20° from centre
  yservo.write(servoPos);
}