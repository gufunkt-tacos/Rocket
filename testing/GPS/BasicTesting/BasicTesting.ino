#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

TinyGPSPlus gps;
HardwareSerial GPSSerial(1); // UART1

void setup() {
  Serial.begin(115200);
  GPSSerial.begin(9600, SERIAL_8N1, RX, TX); // baud, config, RX pin, TX pin
}

void loop() {
  while (GPSSerial.available() > 0) {
    gps.encode(GPSSerial.read());
  }

  if (gps.location.isUpdated()) {
    Serial.print(gps.location.lat(), 6);
    Serial.print(",");
    Serial.println(gps.location.lng(), 6);
  }
}