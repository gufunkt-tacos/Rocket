#include <WiFi.h>
#include <WebServer.h>

const char* ssid     = "Rocket";
const char* password = "Retro:7890";

WebServer server(80);

void handleRoot() {
  String html = "<!DOCTYPE html><html>";
  html += "<head><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>body { font-family: Helvetica; text-align: center; color: white; }";
  html += ".button { background-color: DarkGreen ; border: none; color: white; padding: 16px 40px; font-size: 20px; margin: 2px; cursor: pointer; }</style>";
  html += "</head><body bgcolor='black'>";
  html += "<h1>ESP32 Web Server</h1>";
  html += "<h2>Get Current Date/Time from Phone</h2>";
  html += "<button id='datetimeButton' class='button' onclick='sendDateTime()'>Send Date/Time</button>";
  html += "<p id='status'></p>";
  html += "<script>"
          "function sendDateTime() {"
          "  const now = new Date();"
          "  const year = now.getFullYear();"
          "  const month = String(now.getMonth()+1).padStart(2,'0');"
          "  const day = String(now.getDate()).padStart(2,'0');"
          "  const hours = String(now.getHours()).padStart(2,'0');"
          "  const minutes = String(now.getMinutes()).padStart(2,'0');"
          "  const seconds = String(now.getSeconds()).padStart(2,'0');"
          "  const localDateTime = `${year}-${month}-${day}_${hours}:${minutes}:${seconds}`;"
          "  fetch('/datetime?value=' + encodeURIComponent(localDateTime))"
          "    .then(response => response.text())"
          "    .then(data => {"
          "      document.getElementById('status').innerText = 'ESP32 received: ' + data;"
          "      document.getElementById('datetimeButton').style.backgroundColor = 'maroon';"
          "      document.getElementById('datetimeButton').disabled = true;" // optional
          "    })"
          "    .catch(err => console.error(err));"
          "}"
          "</script>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleDateTime() {
  if (server.hasArg("value")) {
    String dateTime = server.arg("value");
    Serial.println("Received date/time from phone: " + dateTime);
    server.send(200, "text/plain", dateTime);
  } else {
    server.send(400, "text/plain", "Missing value");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Setting up AP...");
  WiFi.softAP(ssid, password);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/datetime", handleDateTime);
  server.begin();
}

void loop() {
  server.handleClient();
}
