#include <Arduino.h>
#include <BlynkSimpleEsp32.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include "time.h"
#include "Adafruit_VL53L1X.h"

#define SWITCH_PIN 0
#define REED_PIN 1
#define RELAY_PIN 2
#define TIMEOUT 30000
int threshold = 900;

bool lightOn = false;
bool readyToRead = true;
bool doorLeftOpen = false;
int16_t distance;

Adafruit_VL53L1X vl53 = Adafruit_VL53L1X();

const char* ssid = "mikesnet";
const char* password = "springchicken";

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -18000;  //Replace with your GMT offset (secs)
const int daylightOffset_sec = 0;   //Replace with your daylight offset (secs)
int hours, mins, secs;
unsigned long triggerTime, debounceTime;

char auth[] = "z5Ja7q52QevSNXIIaUGmsP4pKjS_Nz-_";

AsyncWebServer server(80);

WidgetTerminal terminal(V10);

#define every(interval) \
    static uint32_t __every__##interval = millis(); \
    if (millis() - __every__##interval >= interval && (__every__##interval = millis()))

BLYNK_WRITE(V10) {
  if (String("help") == param.asStr()) {
    terminal.println("==List of available commands:==");
    terminal.println("wifi");
    terminal.println("==End of list.==");
  }
  if (String("wifi") == param.asStr()) {
    terminal.print("Connected to: ");
    terminal.println(ssid);
    terminal.print("IP address:");
    terminal.println(WiFi.localIP());
    terminal.print("Signal strength: ");
    terminal.println(WiFi.RSSI());
    printLocalTime();
  }
    terminal.flush();
}

void printLocalTime() {
  time_t rawtime;
  struct tm* timeinfo;
  time(&rawtime);
  timeinfo = localtime(&rawtime);
  terminal.print(asctime(timeinfo));
}

void setup(void) {
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(REED_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  pinMode(8, OUTPUT);
  Serial.begin(9600);
  delay(1000);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    digitalWrite(8, HIGH);
    delay(250);
    digitalWrite(8, LOW);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! I am ESP32.");
  });

  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
  Serial.println("HTTP server started");
  delay(250);
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  Blynk.config(auth, IPAddress(192, 168, 50, 197), 8080);
  Blynk.connect();
  delay(250);
  struct tm timeinfo;
  getLocalTime(&timeinfo);
  hours = timeinfo.tm_hour;
  mins = timeinfo.tm_min;
  secs = timeinfo.tm_sec;
  terminal.println("***SERVER STARTED***");
  terminal.print("Connected to ");
  terminal.println(ssid);
  terminal.print("IP address: ");
  terminal.println(WiFi.localIP());
  printLocalTime();

  Wire.begin();
  if (! vl53.begin(0x29, &Wire)) {
    //terminal.print(F("Error on init of VL sensor: "));
    //terminal.println(vl53.vl_status);
    while (1)       delay(10);
  }
  terminal.println(F("VL53L1X sensor OK!"));

  terminal.print(F("Sensor ID: 0x"));
  terminal.println(vl53.sensorID(), HEX);

  if (! vl53.startRanging()) {
    terminal.print(F("Couldn't start ranging: "));
    terminal.println(vl53.vl_status);
    while (1)       delay(10);
  }
  terminal.println(F("Ranging started"));

  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  vl53.setTimingBudget(100);
  terminal.print(F("Timing budget (ms): "));
 terminal.println(vl53.getTimingBudget());

  terminal.flush();
}

void loop() {

  

  if (vl53.dataReady()) {
    // new measurement for the taking!
    distance = vl53.distance();
     Serial.println(distance);
    vl53.clearInterrupt();
    if (((distance > 0) && (distance < threshold)) && (lightOn == false)) {
      lightOn = true;
      digitalWrite(RELAY_PIN, HIGH);
      terminal.println("Triggered by motion");
      terminal.flush();
      triggerTime = millis();
    }
  }


  if ((digitalRead(SWITCH_PIN)) && (lightOn == true) && (readyToRead) && (millis() - debounceTime > 100)) {
      lightOn = false;
      debounceTime = millis();
      digitalWrite(RELAY_PIN, LOW);
      terminal.println("Triggered by switch to turn OFF");
      terminal.flush();
      readyToRead = false;
  }

  if ((digitalRead(SWITCH_PIN)) && (lightOn == false) && (readyToRead) && (millis() - debounceTime > 100)) {
      lightOn = true;
      digitalWrite(RELAY_PIN, HIGH);
      terminal.println("Triggered by switch to turn ON");
      terminal.flush();
      debounceTime = millis();
      triggerTime = millis();
      readyToRead = false;
  }

  if (!digitalRead(SWITCH_PIN) && (!readyToRead)) {
    readyToRead = true;
  }



  if (!(digitalRead(REED_PIN)) && (lightOn == false) && (readyToRead) && (!doorLeftOpen) && (millis() - debounceTime > 100)) {
      lightOn = true;
      digitalWrite(RELAY_PIN, HIGH);
      terminal.println("Triggered by reed switch");
      terminal.flush();
      debounceTime = millis();
      triggerTime = millis();
      readyToRead = false;
  }


  if ((millis() - triggerTime > TIMEOUT) && (lightOn) && (distance > threshold)) {
      lightOn = false;
      terminal.println("Light timed out");
      terminal.flush();
      digitalWrite(RELAY_PIN, LOW);
      if (!(digitalRead(REED_PIN)) {doorLeftOpen = true;}
  }

  if ((doorLeftOpen) && (digitalRead(REED_PIN)) {
      doorLeftOpen = false;
  }

      if (WiFi.status() == WL_CONNECTED) {Blynk.run();}
      every(1000){
      Blynk.virtualWrite(V1, distance);
      Blynk.virtualWrite(V2, WiFi.RSSI());
      }


}