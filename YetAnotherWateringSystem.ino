#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>


// Settings.
const char* ssid = "Vodafone-66381823";
const char* password = "supergigi12345678901234567890";
const char* hostname = "terrazzo";
const uint16_t DHTPin = D3; // ESP8266 GPIO pin to use. Using D3, since it haves a build-in pull-up resistor in the D1 mini.

MDNSResponder mdns;
ESP8266WebServer server(80);

void setup(void) {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Serial.
  Serial.begin(115200);
  Serial.println("Booting");

  // WiFi.
  WiFi.hostname(hostname);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(15000);
    ESP.restart();
  }
  Serial.printf("\nConnected to %s, IP address %s\n", ssid, WiFi.localIP().toString().c_str());

  // mDNS.
  if (mdns.begin(hostname, WiFi.localIP()))
    Serial.println("MDNS responder started");

  // OTA.
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.begin();

  //server.on("/ir", handleIr);
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server started");

}

void loop(void) {
  ArduinoOTA.handle();
  server.handleClient();
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}
