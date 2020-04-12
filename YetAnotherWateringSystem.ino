#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <ArduinoOTA.h>
#include <DHT.h>
#include <arduino_secrets.h> // Define your SSID, password and Blynk token here.

// Settings.
#define DHTPIN D3 // GPIO pin to use. Using D3, since it haves a build-in pull-up resistor in the D1 mini board.
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);
int wateringAt = 7 * 60 * 60; // Seconds since midnight.
int lowerThreshold = 20;
BlynkTimer timer;

BLYNK_CONNECTED() {
  // Request Blynk server to re-send latest values.
  Blynk.syncVirtual(V20, V50, V51, V60, V61, V62, V63);
}

/*
   Blynk virtual pins:
    V0: Air temperature from DHT22;
    V1: Air humidity from DHT22;
    V2: Computed dew point;
    V11: Soil moisture of irrigation line 1;
    V12: Soil moisture of irrigation line 2;
    V13: Soil moisture of irrigation line 3;
    V14: Soil moisture of irrigation line 4;
    V20: Button used to force watering;
    V50: Watering time;
    V51: Soil moisture lower threshold; an irrigation is triggered when soil moisture is below the threshold at the irrigation time;
    V60: Target soil moisture for irrigation line 1; the irrigations stops when reachs the soil moisture target.
    V61: Target soil moisture for irrigation line 2;
    V62: Target soil moisture for irrigation line 3;
    V63: Target soil moisture for irrigation line 4;
*/

// Controllers.
BLYNK_WRITE(V20) {
  // It works even if the ESP is temporary offline.
  int forceWatering = param.asInt();
  if (forceWatering == 1) {
    waterNow();
    Blynk.virtualWrite(V20, 0);
  }
}

// Settings.
BLYNK_WRITE(V50) {
  wateringAt = param.asInt();
}
BLYNK_WRITE(V51) {
  lowerThreshold = param.asInt();
}

void setup(void) {
  // Switch off built-in led.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Serial.
  Serial.begin(115200);

  // Set timezone and NTP time sync.
  configTime(0, 0, "pool.ntp.org");
  setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 0);

  // DHT.
  dht.begin();

  // WiFi.
  connectToWiFiOrDoEmergencyWatering();

  // Blynk.
  connectToBlynkOrDoEmergencyWatering();

  // OTA.
  ArduinoOTA.setHostname(SECRET_WATERING_SYSTEM_HOSTNAME);
  ArduinoOTA.begin();

  timer.setInterval(5000L, readAirTemperatureAndHumidity);
}

void loop(void) {
  ArduinoOTA.handle();
  Blynk.run();
  timer.run();
}

/*
  void printTime() {
  time_t tnow = time(nullptr);
  Serial.print(String(ctime(&tnow)));
  }
*/

void connectToWiFiOrDoEmergencyWatering() {
  WiFi.hostname(SECRET_WATERING_SYSTEM_HOSTNAME);
  WiFi.mode(WIFI_STA);
  WiFi.begin(SECRET_SSID, SECRET_PASSWORD);
  float timeout = millis() + 15000;
  while (WiFi.status() != WL_CONNECTED && (timeout - millis() > 0)) {
    delay(1000);
  }
  if (WiFi.status() != WL_CONNECTED) {
    emergencyWateringAndRestart();
  }
}

void connectToBlynkOrDoEmergencyWatering() {
  Blynk.config(SECRET_WATERING_SYSTEM_BLYNK_TOKEN);
  float timeout = millis() + 15000;
  while (Blynk.connect() == false && (timeout - millis() > 0)) {}

  if (!Blynk.connected()) {
    emergencyWateringAndRestart();
  }
}

void emergencyWateringAndRestart() {
  Serial.println("Do an emergency watering, even if the board is disconnected from the Internet.");
  waterNow();

  Serial.println("Restart in 1 hour.");
  delay(60 * 60 * 1000);
  ESP.restart();
}

bool waterNow() {
  return true;
}

void readAirTemperatureAndHumidity() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (isnan(h) || isnan(t) ) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  double dewPoint = computeDewPoint(t, h);

  Serial.printf("Temperature: %f, Humidity: %f%%, Dew point: %f\n", t, h, dewPoint);

  // Update Blynk.
  Blynk.virtualWrite(V1, t);
  Blynk.virtualWrite(V2, h);
  Blynk.virtualWrite(V3, dewPoint);
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
