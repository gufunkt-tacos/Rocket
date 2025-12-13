#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>


Adafruit_MPU6050 mpu;

float additiveOffsetX = (- 0.11 - 0.09)/2;
float additiveOffsetY = 0.23;
float additiveOffsetZ = -0.07;
float multiplicativeOffsetX = 9.81 / 9.68; 
float multiplicativeOffsetY = 9.81 / 10.02; /*9.79*/
float multiplicativeOffsetZ = 9.81 / 10.31;  /*10.40*/

float averagingListx[500];
float averagingListy[500];
float averagingListz[500];
double averageX = 0;
double averageY = 0;
double averageZ = 0;
int n = 500;
int i = 0;

void setup() {
  Serial.begin(9600);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip!");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  if (i < n) {
    averagingListx[i] = a.acceleration.x + additiveOffsetX;
    averagingListy[i] = a.acceleration.y + additiveOffsetY;
    averagingListz[i] = a.acceleration.z + additiveOffsetZ;
    i++;
    delay(200);
  } else {
    for (float element : averagingListx) {
      averageX += element;
    }
    for (float element : averagingListy) {
      averageY += element;
    }
    for (float element : averagingListz) {
      averageZ += element;
    }
    averageX /= n;
    averageY /= n;
    averageZ /= n;
    Serial.print("X: ");
    Serial.println(averageX);
    Serial.print("Y: ");
    Serial.println(averageY);
    Serial.print("Z: ");
    Serial.println(averageZ);
  }

}
