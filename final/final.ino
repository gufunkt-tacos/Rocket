#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SD.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include "esp_wifi.h"
#include "esp_log.h"
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

const char* ssid     = "Rocket";
const char* password = "Retro:7890";

WebServer server(80);

wifi_sta_list_t stationList;

const int chipSelect = 21;
File root;

Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;

TinyGPSPlus gps;
HardwareSerial GPSSerial(1);
#define GPS_RX 17
#define GPS_TX 16
double gpsAltitude = 0.0;
double gpsMaxAltitude = 0.0;
bool gpsAltitudeValid = false;

#define SERVO_PITCH_PIN 0
#define SERVO_YAW_PIN   1
#define ALPHA 0.98

#define PARACHUTE_SERVO_PIN  2    // must differ from SERVO_PITCH_PIN(0) and SERVO_YAW_PIN(1)
#define PARACHUTE_OPEN_POS   150
#define PARACHUTE_CLOSED_POS 70
#define PARACHUTE_DROP_THRESHOLD 5.0  // metres of altitude drop before auto-deploy

// LEDC servo parameters (from working test sketch)
#define SERVO_FREQ 50          // 50 Hz for standard servos
#define PWM_RES   12           // 12‑bit resolution (0–4095)
#define MIN_PULSE_US 1000      // 1 ms for 0°
#define MAX_PULSE_US 2000      // 2 ms for 180°

float pitch = 0.0;
float yaw   = 0.0;
float roll  = 0.0;
unsigned long lastTime = 0;

float gyroXoffset = 0.0;
float gyroYoffset = 0.0;
float gyroZoffset = 0.0;

int servoPitchOffset = 0;
int servoYawOffset   = 0;
#define SERVO_PITCH_TRIM 0
#define SERVO_YAW_TRIM   0

bool servoActive = false;

float basePressure = 1013;
float startTime = 0;

int8_t rssiWiFi;
float lossExponent = 2.5;
double distanceWiFi;
int rssiWiFiat1m = -35;

String dateTime;

bool loggingActive = false;
File logFile;
String logFileName = "";
#define LOG_BUFFER_SIZE 512
char logBuffer[LOG_BUFFER_SIZE];
size_t bufferPos = 0;


bool liveUpdateActive = false;

float offsetGyrX = 0, offsetGyrY = 0, offsetGyrZ = 0;

bool accelCorrectionActive = true;

bool armed = false;
bool parachuteOpen = false;
float maxAltitude = 0.0;       // peak altitude recorded while armed (for auto-deploy)

String initError = "";

// -------------------------------------------------------------------
// Helper: set servo angle using direct LEDC (modern ESP32 API)
// -------------------------------------------------------------------
void setServoAngle(int pin, int angle) {
  angle = constrain(angle, 0, 180);
  int pulseUs = map(angle, 0, 180, MIN_PULSE_US, MAX_PULSE_US);
  int periodUs = 1000000 / SERVO_FREQ;                 // 20 000 µs for 50 Hz
  int duty = map(pulseUs, 0, periodUs, 0, (1 << PWM_RES) - 1);
  ledcWrite(pin, duty);
}

void tryInit(String& errorMsg) {
  errorMsg = "";
  if (!bmp.begin(0x76))      { errorMsg = "Could not find a valid BMP280 sensor!"; return; }
  if (!mpu.begin())          { errorMsg = "Failed to find MPU6050 chip!"; return; }
  if (!SD.begin(chipSelect)) { errorMsg = "SD card initialization failed!"; return; }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  basePressure = bmp.readPressure() / 100;

  pinMode(SERVO_PITCH_PIN, OUTPUT);
  pinMode(SERVO_YAW_PIN, OUTPUT);
  pinMode(PARACHUTE_SERVO_PIN, OUTPUT);

  delay(500);

  // --- Attach servos using direct LEDC ---
  if (!ledcAttach(SERVO_PITCH_PIN, SERVO_FREQ, PWM_RES)) {
    errorMsg = "Failed to attach pitch servo";
    return;
  }
  if (!ledcAttach(SERVO_YAW_PIN, SERVO_FREQ, PWM_RES)) {
    errorMsg = "Failed to attach yaw servo";
    return;
  }
  if (!ledcAttach(PARACHUTE_SERVO_PIN, SERVO_FREQ, PWM_RES)) {
    errorMsg = "Failed to attach parachute servo";
    return;
  }

  // Set initial positions
  setServoAngle(SERVO_PITCH_PIN, 90 + SERVO_PITCH_TRIM);
  setServoAngle(SERVO_YAW_PIN, 90 + SERVO_YAW_TRIM);
  setServoAngle(PARACHUTE_SERVO_PIN, PARACHUTE_CLOSED_POS);
  parachuteOpen = false;

  lastTime = micros();
}

// ── IMU ──────────────────────────────────────────────────────────────────────

void calibrateIMU() {
  basePressure = bmp.readPressure() / 100;

  const int samples = 500;
  double sumGX = 0, sumGY = 0, sumGZ = 0;
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sumGX += g.gyro.x;
    sumGY += g.gyro.y;
    sumGZ += g.gyro.z;
    server.handleClient();
    delay(5);
  }
  gyroXoffset = sumGX / samples;
  gyroYoffset = sumGY / samples;
  gyroZoffset = sumGZ / samples;
  offsetGyrX  = gyroXoffset;
  offsetGyrY  = gyroYoffset;
  offsetGyrZ  = gyroZoffset;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  pitch = atan2(a.acceleration.z, sqrt(a.acceleration.x * a.acceleration.x +
                                        a.acceleration.y * a.acceleration.y)) * RAD_TO_DEG;
  yaw   = atan2(a.acceleration.x, sqrt(a.acceleration.z * a.acceleration.z +
                                        a.acceleration.y * a.acceleration.y)) * RAD_TO_DEG;
  roll  = 0.0;

  servoPitchOffset = -(int)pitch;
  servoYawOffset   = -(int)yaw;

  startTime = millis();
  lastTime  = micros();
}

void updateIMU() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  float gyroX = (g.gyro.x - gyroXoffset) * RAD_TO_DEG;
  float gyroY = (g.gyro.y - gyroYoffset) * RAD_TO_DEG;
  float gyroZ = (g.gyro.z - gyroZoffset) * RAD_TO_DEG;

  float accelMag = sqrt(a.acceleration.x * a.acceleration.x +
                        a.acceleration.y * a.acceleration.y +
                        a.acceleration.z * a.acceleration.z);

  float accelPitch = atan2(a.acceleration.z, sqrt(a.acceleration.x * a.acceleration.x +
                                        a.acceleration.y * a.acceleration.y)) * RAD_TO_DEG;
  float accelYaw   = atan2(a.acceleration.x, sqrt(a.acceleration.z * a.acceleration.z +
                                                    a.acceleration.y * a.acceleration.y)) * RAD_TO_DEG;

  // go gyro-only if under thrust OR if accel correction is disabled
  float alpha = (!accelCorrectionActive || accelMag > 19.6) ? 1.0 : ALPHA;

  pitch = alpha * (pitch + gyroX * dt) + (1.0 - alpha) * accelPitch;
  yaw   = alpha * (yaw   + gyroZ * dt) + (1.0 - alpha) * accelYaw;
  roll += gyroY * dt;
}

// ── GPS ──────────────────────────────────────────────────────────────────────

void updateGPS() {
  while (GPSSerial.available() > 0) {
    gps.encode(GPSSerial.read());
  }
  
  // Extract GPS altitude when valid
  if (gps.altitude.isValid()) {
    gpsAltitude = gps.altitude.meters();
    gpsAltitudeValid = true;
    if (gpsAltitude > gpsMaxAltitude) {
      gpsMaxAltitude = gpsAltitude;
    }
  } else {
    gpsAltitudeValid = false;
  }
}

// ── Error page ───────────────────────────────────────────────────────────────

void handleErrorPage() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>body{background-color:black;color:red;text-align:center;font-family:Helvetica;}"
          ".button{border:none;color:white;padding:16px 40px;font-size:20px;margin:2px;cursor:pointer;}</style>";
  html += "</head><body>";
  html += "<h1>Initialization Error</h1>";
  html += "<p id='msg'>" + initError + "</p>";
  html += "<button class='button' style='background-color:DarkRed' onclick='reinit()'>Retry</button>";
  html += "<p id='status'></p>";
  html += "<script>"
          "function reinit() {"
          "  document.getElementById('status').innerText = 'Retrying...';"
          "  fetch('/reinit')"
          "    .then(r => r.text())"
          "    .then(data => {"
          "      if (data === 'OK') {"
          "        window.location.href = '/';"
          "      } else {"
          "        document.getElementById('msg').innerText = data;"
          "        document.getElementById('status').innerText = 'Failed — check connections and retry';"
          "      }"
          "    });"
          "}"
          "</script>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleReinit() {
  tryInit(initError);
  if (initError.isEmpty()) {
    server.on("/",                handleRoot);
    server.on("/datetime",        handleLogging);
    server.on("/calibrate",       handleCalibrate);
    server.on("/data",            handleData);
    server.on("/toggleLive",      handleLiveToggle);
    server.on("/toggleServo",     handleServoToggle);
    server.on("/toggleAccel",     handleAccelToggle);
    server.on("/toggleArm",       handleToggleArm);
    server.on("/toggleParachute", handleToggleParachute);
    server.send(200, "text/plain", "OK");
  } else {
    server.send(200, "text/plain", initError);
  }
}

// ── Main page ────────────────────────────────────────────────────────────────

void handleRoot() {
  String html = "<!DOCTYPE html><html>";
  html += "<head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>"
          "body { font-family: Helvetica; text-align: center; color: white; }"
          ".button {border: none; color: white; padding: 16px 40px; font-size: 20px; margin: 2px; cursor: pointer; }"
          "</style>";
  html += "</head><body bgcolor='black'>";

  html += "<h1>Rocket Web Portal</h1>";
  html += "<h2>Setup</h2>";
  html += "<button id='logginButton' class='button' onclick='sendDateTime()' style='background-color: Red'>Start Logging</button>";
  html += "<p id='status'></p><br>";
  html += "<button id='calibrateButton' class='button' onclick='sendCalibrate()' style='background-color: Orange'>Calibrate</button>";
  html += "<br><br>";
  html += "<button id='servoButton' class='button' onclick='toggleServo()' style='background-color: Purple'>Enable Servo</button>";
  html += "<br><br>";
  html += "<button id='accelButton' class='button' onclick='toggleAccel()' style='background-color: Teal'>Accel Correction ON</button>";
  html += "<br><br>";
  html += "<button id='connectButton' class='button' onclick='toggleLive()' style='background-color: DarkGreen'>Connect</button>";
  html += "<br><br>";
  html += "<button id='armButton' class='button' onclick='toggleArm()' style='background-color: Maroon'>Arm Parachute</button>";
  html += "<br><br>";
  html += "<button id='openCloseParachuteButton' class='button' onclick='toggleParachute()' style='background-color: darkblue'>Open Parachute</button>";
  html += "<br><br>";

  html += "<h2>Live Sensor Data</h2>";
  html += "<p id='time'>Time: </p>";
  html += "<p id='pressure'>Pressure: </p>";
  html += "<p id='altitude'>Altitude: </p>";
  html += "<p id='acc'>Acceleration: </p>";
  html += "<p id='gyro'>Gyro: </p>";
  html += "<p id='angles'>Pitch:  Roll:  Yaw: </p>";
  html += "<p id='wifi'>Wi-Fi: </p>";
  html += "<h2>GPS</h2>";
  html += "<p id='gps'>No fix</p>";
  html += "<p id='gpsalt'>GPS Altitude: </p>";
  html += "<p id='gpstime'>GPS Time: </p>";
  html += "<p id='gpssats'>Satellites: </p>";

  html += "<script>"
          "let logging = false;"

          "function sendDateTime() {"
          "  let url = '/datetime';"
          "  if (!logging) {"
          "    const now = new Date();"
          "    const year = now.getFullYear();"
          "    const month = String(now.getMonth()+1).padStart(2,'0');"
          "    const day = String(now.getDate()).padStart(2,'0');"
          "    const hours = String(now.getHours()).padStart(2,'0');"
          "    const minutes = String(now.getMinutes()).padStart(2,'0');"
          "    const seconds = String(now.getSeconds()).padStart(2,'0');"
          "    const localDateTime = year+'-'+month+'-'+day+'_'+hours+'-'+minutes+'-'+seconds;"
          "    url += '?value=' + encodeURIComponent(localDateTime);"
          "  }"
          "  fetch(url)"
          "    .then(response => response.text())"
          "    .then(data => {"
          "      const btn = document.getElementById('logginButton');"
          "      document.getElementById('status').innerText = data;"
          "      logging = !logging;"
          "      if (logging) {"
          "        btn.innerText = 'Stop Logging';"
          "        btn.style.backgroundColor = 'maroon';"
          "      } else {"
          "        btn.innerText = 'Start Logging';"
          "        btn.style.backgroundColor = 'Red';"
          "      }"
          "    })"
          "    .catch(err => console.error(err));"
          "}"

          "function sendCalibrate() {"
          "  const btn = document.getElementById('calibrateButton');"
          "  btn.innerText = 'Calibrating...';"
          "  btn.style.backgroundColor = 'DarkOrange';"
          "  btn.disabled = true;"
          "  fetch('/calibrate').then(() => {"
          "    btn.innerText = 'Calibrate';"
          "    btn.style.backgroundColor = 'Orange';"
          "    btn.disabled = false;"
          "  });"
          "}"

          "let servoEnabled = false;"
          "function toggleServo() {"
          "  const btn = document.getElementById('servoButton');"
          "  fetch('/toggleServo').then(r=>r.text()).then(() => {"
          "    servoEnabled = !servoEnabled;"
          "    if (servoEnabled) {"
          "      btn.innerText = 'Disable Servo';"
          "      btn.style.backgroundColor = 'Indigo';"
          "    } else {"
          "      btn.innerText = 'Enable Servo';"
          "      btn.style.backgroundColor = 'Purple';"
          "    }"
          "  });"
          "}"

          "let liveUpdate = false;"
          "let updateInterval;"
          "function toggleLive() {"
          "  const btn = document.getElementById('connectButton');"
          "  fetch('/toggleLive').then(resp=>resp.text()).then(console.log);"
          "  liveUpdate = !liveUpdate;"
          "  if (liveUpdate) {"
          "    btn.innerText = 'Disconnect';"
          "    btn.style.backgroundColor = 'maroon';"
          "    updateInterval = setInterval(updateData, 200);"
          "  } else {"
          "    btn.innerText = 'Connect';"
          "    btn.style.backgroundColor = 'DarkGreen';"
          "    clearInterval(updateInterval);"
          "    updateInterval = null;"
          "  }"
          "}"

          "let accelEnabled = true;"
          "function toggleAccel() {"
          "  const btn = document.getElementById('accelButton');"
          "  fetch('/toggleAccel').then(r=>r.text()).then(() => {"
          "    accelEnabled = !accelEnabled;"
          "    if (accelEnabled) {"
          "      btn.innerText = 'Accel Correction ON';"
          "      btn.style.backgroundColor = 'Teal';"
          "    } else {"
          "      btn.innerText = 'Accel Correction OFF';"
          "      btn.style.backgroundColor = 'DarkSlateGray';"
          "    }"
          "  });"
          "}"

          // ── Parachute arm / deploy ──────────────────────────────────────
          // parachuteArmed  – mirrors the server-side `armed` flag
          // parachuteOpen   – mirrors the server-side `parachuteOpen` flag
          "let parachuteArmed = false;"
          "let parachuteOpen  = false;"

          // Recalculate the deploy button label/colour from current state
          "function updateParachuteButton() {"
          "  const btn = document.getElementById('openCloseParachuteButton');"
          "  if (!parachuteArmed) {"
          "    if (parachuteOpen) {"
          "      btn.innerText = 'Close Parachute';"
          "      btn.style.backgroundColor = 'steelblue';"
          "    } else {"
          "      btn.innerText = 'Open Parachute';"
          "      btn.style.backgroundColor = 'darkblue';"
          "    }"
          "  } else {"
          "    if (parachuteOpen) {"
          "      btn.innerText = 'Close Parachute';"
          "      btn.style.backgroundColor = 'steelblue';"
          "    } else {"
          "      btn.innerText = 'Emergency Deploy';"
          "      btn.style.backgroundColor = 'crimson';"
          "    }"
          "  }"
          "}"

          "function toggleArm() {"
          "  const btn = document.getElementById('armButton');"
          "  fetch('/toggleArm').then(r=>r.text()).then(() => {"
          "    parachuteArmed = !parachuteArmed;"
          "    if (parachuteArmed) {"
          "      btn.innerText = 'Disarm Parachute';"
          "      btn.style.backgroundColor = 'Red';"
          "    } else {"
          "      btn.innerText = 'Arm Parachute';"
          "      btn.style.backgroundColor = 'Maroon';"
          "    }"
          "    updateParachuteButton();"
          "  });"
          "}"

          "function toggleParachute() {"
          "  fetch('/toggleParachute').then(r=>r.text()).then(() => {"
          "    parachuteOpen = !parachuteOpen;"
          "    updateParachuteButton();"
          "  });"
          "}"
          // ───────────────────────────────────────────────────────────────

          "function updateData() {"
          "  fetch('/data')"
          "    .then(response => response.json())"
          "    .then(data => {"
          "      document.getElementById('time').innerText = 'Time: ' + data.time.toFixed(2) + ' s';"
          "      document.getElementById('pressure').innerText = 'Pressure: ' + data.pressure + ' hPa';"
          "      document.getElementById('altitude').innerText = 'Altitude: ' + data.altitude + ' m';"
          "      document.getElementById('acc').innerText = 'Acceleration X:' + data.accX.toFixed(2) +"
          "        ' Y:' + data.accY.toFixed(2) + ' Z:' + data.accZ.toFixed(2);"
          "      document.getElementById('gyro').innerText = 'Rotation X:' + data.gyroX.toFixed(2) +"
          "        ' Y:' + data.gyroY.toFixed(2) + ' Z:' + data.gyroZ.toFixed(2);"
          "      document.getElementById('angles').innerText = 'Pitch: ' + data.pitch.toFixed(2) + '°  Roll: ' + data.roll.toFixed(2) + '°  Yaw: ' + data.yaw.toFixed(2) + '°';"
          "      document.getElementById('wifi').innerText = 'Wi-Fi: ' + data.rssiWiFi + ' dBm (' + data.distanceWiFi.toFixed(2) + ' m)';"
          "      if (data.gpsFix) {"
          "        document.getElementById('gps').innerText = 'Lat: ' + data.gpsLat.toFixed(6) + '  Lng: ' + data.gpsLng.toFixed(6);"
          "        document.getElementById('gpsalt').innerText = 'GPS Altitude: ' + data.gpsAlt.toFixed(1) + ' m';"
          "        document.getElementById('gpstime').innerText = 'GPS Time (UTC): ' + data.gpsTime;"
          "        document.getElementById('gpssats').innerText = 'Satellites: ' + data.gpsSats;"
          "      } else {"
          "        document.getElementById('gps').innerText = 'No fix';"
          "        document.getElementById('gpssats').innerText = 'Satellites: ' + data.gpsSats;"
          "      }"
          "    });"
          "}"
          "</script>";

  html += "</body></html>";
  server.send(200, "text/html", html);
}

// ── Handlers ─────────────────────────────────────────────────────────────────

void handleLogging() {
  if (!loggingActive) {
    if (!server.hasArg("value")) {
      server.send(400, "text/plain", "Missing value");
      return;
    }
    dateTime    = server.arg("value");
    logFileName = "/log_" + dateTime + ".csv";
    logFile     = SD.open(logFileName, FILE_WRITE);
    if (!logFile) {
      server.send(500, "text/plain", "Failed to create log file");
      return;
    }
    logFile.println("time,pressure,altitude,accX,accY,accZ,gyroX,gyroY,gyroZ,pitch,roll,yaw,gpsAlt");
    logFile.flush();
    calibrateIMU();
    loggingActive = true;
    server.send(200, "text/plain", "Log started in " + logFileName);
  } else {
    loggingActive = false;
    if (logFile) { logFile.flush(); logFile.close(); }
    server.send(200, "text/plain", "Log file saved.");
  }
}

void handleAccelToggle() {
  accelCorrectionActive = !accelCorrectionActive;
  server.send(200, "text/plain", accelCorrectionActive ? "Accel ON" : "Accel OFF");
}

void handleCalibrate() {
  calibrateIMU();
  server.send(200, "text/plain", "Calibrated");
}

void handleServoToggle() {
  servoActive = !servoActive;
  if (!servoActive) {
    setServoAngle(SERVO_PITCH_PIN, 90 + SERVO_PITCH_TRIM);
    setServoAngle(SERVO_YAW_PIN, 90 + SERVO_YAW_TRIM);
  }
  server.send(200, "text/plain", servoActive ? "Servo ON" : "Servo OFF");
}

void handleToggleArm() {
  armed = !armed;
  if (armed) {
    // capture current altitude as the reference peak when arming
    maxAltitude = bmp.readAltitude(basePressure);
    gpsMaxAltitude = gpsAltitude;
  }
  server.send(200, "text/plain", armed ? "Armed" : "Disarmed");
}

void handleToggleParachute() {
  parachuteOpen = !parachuteOpen;
  setServoAngle(PARACHUTE_SERVO_PIN, parachuteOpen ? PARACHUTE_OPEN_POS : PARACHUTE_CLOSED_POS);
  
  // NEW: When parachute opens, disable stabilization servos and center them
  if (parachuteOpen) {
    servoActive = false;
    setServoAngle(SERVO_PITCH_PIN, 90 + SERVO_PITCH_TRIM);
    setServoAngle(SERVO_YAW_PIN, 90 + SERVO_YAW_TRIM);
  }
  
  server.send(200, "text/plain", parachuteOpen ? "Parachute Open" : "Parachute Closed");
}

void handleData() {
  if (!liveUpdateActive) {
    server.send(200, "application/json", "{}");
    return;
  }

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float pressure = bmp.readPressure() / 100;
  float altitude = bmp.readAltitude(basePressure);
  float timeSec  = (millis() - startTime) / 1000;

  esp_wifi_ap_get_sta_list(&stationList);
  if (stationList.num == 0) {
    rssiWiFi     = 0;
    distanceWiFi = 0;
  } else {
    wifi_sta_info_t sta = stationList.sta[0];
    rssiWiFi     = sta.rssi;
    distanceWiFi = pow(10, (rssiWiFiat1m - rssiWiFi) / (10 * lossExponent));
  }

  // GPS
  bool gpsFix  = gps.location.isValid();
  double gpsLat = gpsFix ? gps.location.lat() : 0.0;
  double gpsLng = gpsFix ? gps.location.lng() : 0.0;
  double gpsAlt = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
  int    gpsSats = gps.satellites.isValid() ? gps.satellites.value() : 0;

  char gpsTimeBuf[12] = "N/A";
  if (gps.time.isValid()) {
    snprintf(gpsTimeBuf, sizeof(gpsTimeBuf), "%02d:%02d:%02d",
             gps.time.hour(), gps.time.minute(), gps.time.second());
  }

  // String json = "{";
  // json += "\"time\":"         + String(timeSec)               + ",";
  // json += "\"pressure\":"     + String(pressure)              + ",";
  // json += "\"altitude\":"     + String(altitude)              + ",";
  // json += "\"accX\":"         + String(a.acceleration.x)      + ",";
  // json += "\"accY\":"         + String(a.acceleration.y)      + ",";
  // json += "\"accZ\":"         + String(a.acceleration.z)      + ",";
  // json += "\"gyroX\":"        + String(g.gyro.x - offsetGyrX) + ",";
  // json += "\"gyroY\":"        + String(g.gyro.y - offsetGyrY) + ",";
  // json += "\"gyroZ\":"        + String(g.gyro.z - offsetGyrZ) + ",";
  // json += "\"pitch\":"        + String(pitch)                 + ",";
  // json += "\"roll\":"         + String(roll)                  + ",";
  // json += "\"yaw\":"          + String(yaw)                   + ",";
  // json += "\"rssiWiFi\":"     + String(rssiWiFi)              + ",";
  // json += "\"distanceWiFi\":" + String(distanceWiFi)          + ",";
  // json += "\"gpsFix\":"       + String(gpsFix ? "true" : "false") + ",";
  // json += "\"gpsLat\":"       + String(gpsLat, 6)             + ",";
  // json += "\"gpsLng\":"       + String(gpsLng, 6)             + ",";
  // json += "\"gpsAlt\":"       + String(gpsAlt, 1)             + ",";
  // json += "\"gpsSats\":"      + String(gpsSats)               + ",";
  // json += "\"gpsTime\":\"" + String(gpsTimeBuf) + "\"";
  // json += "}";

  // server.send(200, "application/json", json);
  char jsonBuffer[512];
  snprintf(jsonBuffer, sizeof(jsonBuffer),
  "{\"time\":%.3f,\"pressure\":%.1f,\"altitude\":%.2f,\"accX\":%.2f,\"accY\":%.2f,\"accZ\":%.2f,"
  "\"gyroX\":%.2f,\"gyroY\":%.2f,\"gyroZ\":%.2f,\"pitch\":%.2f,\"roll\":%.2f,\"yaw\":%.2f,"
  "\"rssiWiFi\":%d,\"distanceWiFi\":%.2f,\"gpsFix\":%s,\"gpsLat\":%.6f,\"gpsLng\":%.6f,"
  "\"gpsAlt\":%.1f,\"gpsSats\":%d,\"gpsTime\":\"%s\"}",
  timeSec, pressure, altitude,
  a.acceleration.x, a.acceleration.y, a.acceleration.z,
  g.gyro.x - offsetGyrX, g.gyro.y - offsetGyrY, g.gyro.z - offsetGyrZ,
  pitch, roll, yaw,
  rssiWiFi, distanceWiFi,
  gpsFix ? "true" : "false", gpsLat, gpsLng, gpsAlt, gpsSats, gpsTimeBuf);

  server.send(200, "application/json", jsonBuffer);
}

void handleLiveToggle() {
  liveUpdateActive = !liveUpdateActive;
  server.send(200, "text/plain", liveUpdateActive ? "Live ON" : "Live OFF");
}


void flushLogBuffer() {
  if (bufferPos > 0 && logFile) {
    logFile.write((const uint8_t*)logBuffer, bufferPos);
    logFile.flush();  // Only flush when buffer is full
    bufferPos = 0;
  }
}

// ── Setup & loop ─────────────────────────────────────────────────────────────

void setup() {
  pinMode(WIFI_ENABLE, OUTPUT);
  digitalWrite(WIFI_ENABLE, LOW);
  delay(100);
  pinMode(WIFI_ANT_CONFIG, OUTPUT);
  digitalWrite(WIFI_ANT_CONFIG, HIGH);

  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  WiFi.softAP(ssid, password);

  server.on("/error",  handleErrorPage);
  server.on("/reinit", handleReinit);

  tryInit(initError);

  if (initError.isEmpty()) {
    server.on("/",                handleRoot);
    server.on("/datetime",        handleLogging);
    server.on("/calibrate",       handleCalibrate);
    server.on("/data",            handleData);
    server.on("/toggleLive",      handleLiveToggle);
    server.on("/toggleAccel",     handleAccelToggle);
    server.on("/toggleServo",     handleServoToggle);
    server.on("/toggleArm",       handleToggleArm);
    server.on("/toggleParachute", handleToggleParachute);
  } else {
    server.onNotFound([]() {
      server.sendHeader("Location", "/error");
      server.send(302, "text/plain", "");
    });
  }

  server.begin();
}

void loop() {
  static uint32_t lastServerHandle = 0;
  if (millis() - lastServerHandle > 50) {  // Max 20 Hz for web
    server.handleClient();
    lastServerHandle = millis();
  }

  updateGPS();

  if (initError.isEmpty()) {
    // ── Read sensors ONCE ───────────────────────────────────────────────────
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float pressure   = bmp.readPressure() / 100;
    float altitude   = bmp.readAltitude(basePressure);
    float timeSec    = (millis() - startTime) / 1000.0;

    updateIMU();

    if (servoActive) {
      int servoPitchPos = constrain(90 + (int)pitch + servoPitchOffset, 45, 135) + SERVO_PITCH_TRIM;
      int servoYawPos   = constrain(90 + (int)yaw   + servoYawOffset,   45, 135) + SERVO_YAW_TRIM;
      setServoAngle(SERVO_PITCH_PIN, servoPitchPos);
      setServoAngle(SERVO_YAW_PIN, servoYawPos);
    }

    // ── Auto-deploy parachute ─────────────────────────────────────────────────
    if (armed && !parachuteOpen) {
      if (altitude > maxAltitude) {
        maxAltitude = altitude;      
      } else if (((maxAltitude - altitude) >= PARACHUTE_DROP_THRESHOLD) || (gpsAltitudeValid && ((gpsMaxAltitude - gpsAltitude) >= PARACHUTE_DROP_THRESHOLD))) {
        setServoAngle(PARACHUTE_SERVO_PIN, PARACHUTE_OPEN_POS);
        
        // NEW: When parachute auto‑deploys, disable stabilization and center servos
        parachuteOpen = true;
        servoActive = false;
        setServoAngle(SERVO_PITCH_PIN, 90 + SERVO_PITCH_TRIM);
        setServoAngle(SERVO_YAW_PIN, 90 + SERVO_YAW_TRIM);
      }
    }
    // ─────────────────────────────────────────────────────────────────────────

    // if (loggingActive && logFile) {
    //   float timeSec = (millis() - startTime) / 1000.0;
    //   sensors_event_t a, g, temp;
    //   mpu.getEvent(&a, &g, &temp);

    //   logFile.print(timeSec);                        logFile.print(",");
    //   logFile.print(bmp.readPressure() / 100);       logFile.print(",");
    //   logFile.print(bmp.readAltitude(basePressure)); logFile.print(",");
    //   logFile.print(a.acceleration.x);               logFile.print(",");
    //   logFile.print(a.acceleration.y);               logFile.print(",");
    //   logFile.print(a.acceleration.z);               logFile.print(",");
    //   logFile.print(g.gyro.x - offsetGyrX);          logFile.print(",");
    //   logFile.print(g.gyro.y - offsetGyrY);          logFile.print(",");
    //   logFile.print(g.gyro.z - offsetGyrZ);          logFile.print(",");
    //   logFile.print(pitch);                          logFile.print(",");
    //   logFile.print(roll);                           logFile.print(",");
    //   logFile.print(yaw);
    //   logFile.print("\r\n");
    //   logFile.flush();
    // }

    if (loggingActive && logFile) {
      int written = snprintf(logBuffer + bufferPos, LOG_BUFFER_SIZE - bufferPos,
        "%.3f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
        timeSec, pressure, altitude,
        a.acceleration.x, a.acceleration.y, a.acceleration.z,
        g.gyro.x - offsetGyrX, g.gyro.y - offsetGyrY, g.gyro.z - offsetGyrZ,
        pitch, roll, yaw,
        gpsAltitude);
      
      bufferPos += written;
      if (bufferPos >= LOG_BUFFER_SIZE - 100) flushLogBuffer();
    }

  }
}




