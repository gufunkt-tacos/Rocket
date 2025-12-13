#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;

float basePressure;
float additiveOffsetAccX = (- 0.11 - 0.09)/2;
float additiveOffsetAccY = 0.23;
float additiveOffsetAccZ = -0.07;
float multiplicativeOffsetGyroX = 9.81 / 9.68; 
float multiplicativeOffsetGyroY = 9.81 / 10.02;
float multiplicativeOffsetGyroZ = 9.81 / 10.31;
float additiveOffsetGyroX = 0.08;
float additiveOffsetGyroY = 0.01;
float additiveOffsetGyroZ = -0.01;

void setup() {
  Serial.begin(9600);

  if  (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor!"));
    while (1){
      delay(10);
    }
  }

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

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
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

  Serial.println("");

  /* Default settings from datasheet.  */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500);  /* Standby time. */

  basePressure = bmp.readPressure() / 100;    

    
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude(basePressure));
  Serial.println(" m\r");   
  
  Serial.print("Acceleration X: ");
  Serial.print((a.acceleration.x + additiveOffsetAccX)*multiplicativeOffsetGyroX);
  Serial.print(", Y: ");
  Serial.print((a.acceleration.y + additiveOffsetAccY)*multiplicativeOffsetGyroY);
  Serial.print(", Z: ");
  Serial.print((a.acceleration.z + additiveOffsetAccZ)*multiplicativeOffsetGyroZ);
  Serial.println(" m s^-2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x + additiveOffsetGyroX);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y + additiveOffsetGyroY);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z+ additiveOffsetGyroZ);
  Serial.println(" rad s^-1");

  delay(100);

  Serial.print("\033[1A");
  Serial.print("\033[2K");
  Serial.print("\033[1A");
  Serial.print("\033[2K");
  Serial.print("\033[1A");
  Serial.print("\033[2K");
}
