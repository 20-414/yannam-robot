#define TINY_GSM_MODEM_SIM7600  // SIM7600 AT instruction is compatible with A7670
#define SerialAT Serial1
#define SerialMon Serial
#define TINY_GSM_USE_GPRS true

#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include "EEPROM.h"
#include "esp_system.h"
#include <TimeLib.h>
#include <ESP32Time.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include "esp_wifi.h"
#include "esp_mac.h"

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme; // I2C

// VVM501 Pin Mappings
#define RXPin 16
#define TXPin 17
#define powerPin 4
#define RXD2 27
#define TXD2 26

RTC_DATA_ATTR int numStoredPackets = 0;
RTC_DATA_ATTR int bootCount = 0;

ESP32Time rtc(3600);  // offset in seconds GMT+1
TinyGPSPlus gps;
HardwareSerial GPSSerial(2);

String rxString;
int rx = -1;
int _timeout;
String _buffer;
String jsonPayload;
char jsonStr[512];
uint8_t mac[6];
bool storeInEEprom = false;

const int sleepTimeSeconds = 90;
#define EEPROM_SIZE 4096
#define MAX_PACKET_SIZE 200
#define EEPROM_PREF_INDEX (EEPROM_SIZE - 2)
const char apn[] = ""; // Leave blank for auto detection

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

TinyGsmClient client(modem);
PubSubClient mqtt(client);

// ====== Setup Function ======
void setup() {
  Serial.begin(115200);
  SerialAT.begin(115200, SERIAL_8N1, RXD2, TXD2);
  delay(3000);

  Serial.println("Initializing modem...");
  if (!modem.init()) {
    Serial.println("Failed to init modem");
    delay(10000);
    return;
  }

  if (!modem.restart()) {
    Serial.println("Failed to restart modem");
    delay(10000);
    return;
  }

  Serial.println("Waiting for network...");
  if (!modem.waitForNetwork()) {
    Serial.println("Network initialization failed.");
    delay(5000);
    return;
  }
  Serial.println("Success");

  if (modem.isNetworkConnected()) {
    Serial.println("Network connected");
  }

  Serial.print(F("Connecting to GPRS..."));
  if (!modem.gprsConnect(apn)) {
    Serial.println("GPRS connection failed.");
    delay(5000);
    return;
  }
  Serial.println("Success");

  if (modem.isGprsConnected()) {
    Serial.println("LTE module connected");
  }

  Serial.println("GPS module initialized");

  // Get and apply epoch time from SIM
  long epochTime = getTimeFromSIMModule();
  Serial.print("Epoch time from SIM Module: ");
  Serial.println(epochTime);
  rtc.setTime(epochTime);
}

// ====== Loop Function ======
void loop() {
  if (!modem.isNetworkConnected()) {
     Serial.println("Network disconnected");
  } //else {
  //  Serial.println("Sensor data published to MQTT");
 // }

 // delay(10000);  // Add a delay to avoid flooding serial output
}

// ====== Get SIM Timestamp and Convert ======
long getTimeFromSIMModule() {
  String timestamp;
  SerialAT.println("AT+CCLK?");
  delay(300);

  while (SerialAT.available()) {
    char c = SerialAT.read();
    timestamp += c;
  }

  Serial.println("Raw timestamp from SIM Module: " + timestamp);

  int timestampIndex = timestamp.indexOf("+CCLK: ");
  if (timestampIndex != -1) {
    timestamp.remove(0, timestampIndex + 7);
    timestamp.trim();

    int timestampStart = timestamp.indexOf('\"');
    int timestampEnd = timestamp.lastIndexOf('\"');

    if (timestampStart != -1 && timestampEnd != -1) {
      String ts = timestamp.substring(timestampStart + 1, timestampEnd);
      return convertTimestampToEpoch(ts);
    }
  }

  Serial.println("Failed to parse SIM time, using RTC time");
  return rtc.getLocalEpoch();
}

// ====== Convert to Epoch ======
long convertTimestampToEpoch(const String &timestamp) {
  int year, month, day, hour, minute, second;
  sscanf(timestamp.c_str(), "%d/%d/%d,%d:%d:%d", &year, &month, &day, &hour, &minute, &second);
  year += 2000;

  struct tm tm;
  tm.tm_year = year - 1900;
  tm.tm_mon = month - 1;
  tm.tm_mday = day;
  tm.tm_hour = hour;
  tm.tm_min = minute;
  tm.tm_sec = second;

  time_t epochTime = mktime(&tm);
  if (epochTime == -1) {
    Serial.println("Failed to convert to epoch");
    return rtc.getLocalEpoch();
  }

  return epochTime;
}

// ====== MQTT Callback (if needed) ======
void callback(char* topic, byte* payload, unsigned int length) {
  // Handle MQTT messages here
}
