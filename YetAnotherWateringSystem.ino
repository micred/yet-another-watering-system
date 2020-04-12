#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <ArduinoOTA.h>
#include <DHT.h>
#include <arduino_secrets.h> // Define your SSID, password and Blynk token here.

// Settings.
#define DHTPIN D3 // GPIO pin to use. Using D3, since it haves a build-in pull-up resistor in the D1 mini board.
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);
BlynkTimer timer;

void setup(void) {
  // Switch off built-in led.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Serial.
  Serial.begin(115200);

  // Set timezone and NTP time sync.
  configTime(0, 0, "pool.ntp.org");
  setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 0);

  // WiFi.
  Blynk.begin(SECRET_WATERING_SYSTEM_BLYNK_TOKEN, SECRET_SSID, SECRET_PASSWORD);

  // OTA.
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.setHostname(SECRET_WATERING_SYSTEM_HOSTNAME);
  ArduinoOTA.begin();

  // DHT.
  dht.begin();

  // Setup a function to be called every second.
  timer.setInterval(1000L, readTemperatureAndHumidity);
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

void readTemperatureAndHumidity() {
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
