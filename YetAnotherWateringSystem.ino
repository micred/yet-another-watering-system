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

#if defined(ESP32)
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#endif
#include <RemoteDebug.h>
#include <ArduinoOTA.h>
#include <DHT.h>
#include <arduino_secrets.h> // Define your SSID, password, hostname and Blynk token inside arduino_secrets.h

typedef struct {
  bool enabled;
  uint8_t soilMoistureSensorPin;
  uint8_t valvePin;
  uint8_t upperThreshold;
} irrigationLine;

// Settings.
#define DHTTYPE DHT22
#define DHTPIN GPIO_NUM_13 // GPIO pin where DHT is connected. Use a pull-up resistor if your board doesn't have an internal one.
#define PWSAVERPIN GPIO_NUM_32 // Enable power rail (12v/3.3v) driven by a MOSFET (used to switch off external sensors and save battery). 
uint8_t wateringAt = 7 * 60 * 60; // Seconds since midnight.
uint8_t lowerThreshold = 20;

irrigationLine irrigationLines[] {
  // {enabled, soilMoistureSensorPin, valvePin, upperThreshold}
  {true, GPIO_NUM_12, GPIO_NUM_16, 50},
  {true, GPIO_NUM_14, GPIO_NUM_17, 50},
  {true, GPIO_NUM_27, GPIO_NUM_5, 50},
  {true, GPIO_NUM_26, GPIO_NUM_18, 50}
};

BLYNK_CONNECTED() {
  // Request Blynk server to re-send latest values.
  Blynk.syncVirtual(
    V20,               // Button used to force watering;
    V50,               // Watering time;
    V51,               // Soil moisture lower threshold;
    V60, V61, V62, V63 // Target soil moisture for irrigation line 1-4
  );
}

// Callbacks used for Blynk controllers.
BLYNK_WRITE(V20) {
  // It works even if the ESP is temporary offline.
  int forceWatering = param.asInt();
  if (forceWatering == 1) {
    waterNow();
    Blynk.virtualWrite(V20, 0);
  }
}

// Callbacks used for updating settings from Blynk to ESP.
BLYNK_WRITE(V50) {
  wateringAt = param.asInt();
}
BLYNK_WRITE(V51) {
  lowerThreshold = param.asInt();
}
BLYNK_WRITE(V60) {
  setUpperThreshold(0, param.asInt());
}
BLYNK_WRITE(V61) {
  setUpperThreshold(1, param.asInt());
}
BLYNK_WRITE(V62) {
  setUpperThreshold(2, param.asInt());
}
BLYNK_WRITE(V63) {
  setUpperThreshold(3, param.asInt());
}

RemoteDebug Debug;
DHT dht(DHTPIN, DHTTYPE);
BlynkTimer timer;

void setup(void) {
  // Switch off built-in led.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(PWSAVERPIN, OUTPUT);
  digitalWrite(PWSAVERPIN, HIGH);

  setValvesAndMoistureSensorsPinModes();

  // Serial.
  Serial.begin(115200);

  // DHT.
  dht.begin();

  // WiFi.
  connectToWiFiOrDoEmergencyWatering();

  // Blynk.
  connectToBlynkOrDoEmergencyWatering();

  // Remote debug.
  Debug.begin(SECRET_GAS_SENSOR_HOSTNAME);
  Debug.setSerialEnabled(true);
  Debug.setResetCmdEnabled(true);
  Debug.showColors(true);

  // OTA.
  ArduinoOTA.setHostname(SECRET_WATERING_SYSTEM_HOSTNAME);
  ArduinoOTA.begin();

  // Set timezone and NTP time sync.
  configTime(0, 0, "pool.ntp.org");
  setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 0);

  //  Enable light sleep.
  //wifi_set_sleep_type(LIGHT_SLEEP_T);
  //https://community.blynk.cc/t/esp8266-light-sleep/13584

  // Periodic jobs.
  timer.setInterval(5000L, readAirTemperatureAndHumidity);
}

void loop(void) {
  Debug.handle();
  ArduinoOTA.handle();
  Blynk.run();
  timer.run();
  //delay(1000);  // Put the ESP to sleep for 1s. https://community.blynk.cc/t/esp8266-light-sleep/13584
}

bool setUpperThreshold(uint8_t irrigationLineIdx, uint8_t newThreshold) {
  if (irrigationLineIdx >= sizeof(irrigationLines)) {
    return false;
  }
  irrigationLines[irrigationLineIdx].upperThreshold = newThreshold;
  return true;
}

void setValvesAndMoistureSensorsPinModes() {
  for (uint8_t i = 0; i < sizeof(irrigationLines) / sizeof(irrigationLine); i++) {
    pinMode(irrigationLines[i].soilMoistureSensorPin, INPUT);
    pinMode(irrigationLines[i].valvePin, OUTPUT);
  }
}

void connectToWiFiOrDoEmergencyWatering() {
  #if defined(ESP32)
  WiFi.setHostname(SECRET_WATERING_SYSTEM_HOSTNAME);
  #elif defined(ESP8266)
  WiFi.hostname(SECRET_WATERING_SYSTEM_HOSTNAME);
  #endif
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
  Debug.println("Do an emergency watering, even if the board is disconnected from the Internet.");
  waterNow();

  Debug.println("Restart in 1 hour.");
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
    Debug.println(F("Failed to read from DHT sensor!"));
    return;
  }
  double dewPoint = computeDewPoint(t, h);

  Debug.printf("Temperature: %f, Humidity: %f%%, Dew point: %f\n", t, h, dewPoint);

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
