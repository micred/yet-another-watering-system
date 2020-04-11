#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <DHT.h>

// Settings.
#define DHTPIN D3 // ESP8266 GPIO pin to use. Using D3, since it haves a build-in pull-up resistor in the D1 mini.
#define DHTTYPE DHT22
const char* ssid = "Vodafone-66381823";
const char* password = "supergigi12345678901234567890";
const char* hostname = "terrazzo";
//TODO test https://github.com/tzapu/WiFiManager#wifimanager ?

MDNSResponder mdns;
ESP8266WebServer server(80);
DHT dht(DHTPIN, DHTTYPE);

void setup(void) {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Serial.
  Serial.begin(115200);
  Serial.println("Booting");

  // Time.
  configTime(0, 0, "pool.ntp.org");
  setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 0);

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

  // DHT.
  dht.begin();
}

void loop(void) {
  ArduinoOTA.handle();
  readTemperatureAndHumidity();
  printTime();
  delay(2000);
  //server.handleClient();
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

void printTime() {
  time_t tnow = time(nullptr);
  Serial.print(String(ctime(&tnow)));
}

void readTemperatureAndHumidity() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (isnan(h) || isnan(t) ) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  double dewPoint = computeDewPoint(t, h);

  Serial.printf("Temperature: %f, Humidity: %f%%, Dew point: %f\n", t, h, dewPoint);
}

// reference: http://wahiduddin.net/calc/density_algorithms.htm
double computeDewPoint(double celsius, double humidity) {
  double RATIO = 373.15 / (273.15 + celsius);  // RATIO was originally named A0, possibly confusing in Arduino context
  double SUM = -7.90298 * (RATIO - 1);
  SUM += 5.02808 * log10(RATIO);
  SUM += -1.3816e-7 * (pow(10, (11.344 * (1 - 1 / RATIO ))) - 1) ;
  SUM += 8.1328e-3 * (pow(10, (-3.49149 * (RATIO - 1))) - 1) ;
  SUM += log10(1013.246);
  double VP = pow(10, SUM - 3) * humidity;
  double T = log(VP / 0.61078); // temp var
  return (241.88 * T) / (17.558 - T);
}
