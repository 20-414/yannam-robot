#define TINY_GSM_MODEM_SIM7600
#include <TinyGsmClient.h>
#include <ArduinoJson.h>
#include <TimeLib.h>
#include <ESP32Time.h>

#define SerialAT Serial1
#define SerialMon Serial
#define TINY_GSM_USE_GPRS true

const char apn[] = "";
ESP32Time rtc;

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

void setup() {
  SerialMon.begin(115200);
  delay(1000);
  SerialAT.begin(115200, SERIAL_8N1, 27, 26);  // RXD2, TXD2
  delay(3000);

  SerialMon.println("Initializing modem...");
  if (!modem.init()) {
    SerialMon.println("Failed to init modem");
    return;
  }

  if (!modem.restart()) {
    SerialMon.println("Failed to restart modem");
    return;
  }

  SerialMon.println("Waiting for network...");
  if (!modem.waitForNetwork()) {
    SerialMon.println("Network initialization failed.");
    return;
  }
  SerialMon.println("Success");

  if (modem.isNetworkConnected()) {
    SerialMon.println("Network connected");
  }

  SerialMon.print(F("Connecting to GPRS..."));
  if (!modem.gprsConnect(apn)) {
    SerialMon.println("GPRS connection failed.");
    return;
  }
  SerialMon.println("Success");

  if (modem.isGprsConnected()) {
    SerialMon.println("LTE module connected");
  }

  SerialMon.println("GPS module initialized");

  long epoch = getTimeFromSIMModule();
  rtc.setTime(epoch);

  // Build JSON output
  StaticJsonDocument<200> doc;
  doc["epoch"] = epoch;
  serializeJson(doc, SerialMon);
  SerialMon.println(); // new line
}

void loop() {
  delay(10000);
}

long getTimeFromSIMModule() {
  String timestamp;
  SerialAT.println("AT+CCLK?");
  delay(300);
  while (SerialAT.available()) {
    char c = SerialAT.read();
    timestamp += c;
  }
Serial.println(timestamp);

  int idx = timestamp.indexOf("+CCLK: ");
  if (idx != -1) {
    timestamp.remove(0, idx + 7);
    timestamp.trim();
    int s = timestamp.indexOf('\"');
    int e = timestamp.lastIndexOf('\"');
    if (s != -1 && e != -1) {
      String ts = timestamp.substring(s + 1, e);
      return convertToEpoch(ts);
    }
  }

  SerialMon.println("Failed to parse SIM time, using RTC time");
  return rtc.getLocalEpoch();
}

long convertToEpoch(const String &timestamp) {
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
  return mktime(&tm);
}
