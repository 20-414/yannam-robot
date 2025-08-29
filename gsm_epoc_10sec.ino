#define TINY_GSM_MODEM_SIM7600
#include <TinyGsmClient.h>
#include <ArduinoJson.h>
#include <TimeLib.h>
#include <ESP32Time.h>

#define SerialAT Serial1
#define SerialMon Serial
#define TINY_GSM_USE_GPRS true

const char apn[] = "";  // Leave empty for auto
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

  // Get time from SIM and set RTC
  long epoch = getTimeFromSIMModule();
  rtc.setTime(epoch);

  SerialMon.println("Start streaming timestamp JSON every 10s...");
}

void loop() {
  long epoch = rtc.getEpoch();
  String readableTime = rtc.getTime("%Y-%m-%d %H:%M:%S");

  StaticJsonDocument<256> doc;
  doc["epoch"] = epoch;
  doc["timestamp"] = readableTime;

  serializeJson(doc, SerialMon);
  SerialMon.println();

  delay(10000);  // Print every 10 seconds
}

// ========== Get Epoch from SIM ==========
long getTimeFromSIMModule() {
  String timestamp;
  SerialAT.println("AT+CCLK?");
  delay(300);

  while (SerialAT.available()) {
    char c = SerialAT.read();
    timestamp += c;
  }

  SerialMon.println("Raw SIM timestamp: " + timestamp);

  int idx = timestamp.indexOf("+CCLK: ");
  if (idx != -1) {
    timestamp.remove(0, idx + 7);
    timestamp.trim();

    int start = timestamp.indexOf('"');
    int end = timestamp.lastIndexOf('"');
    if (start != -1 && end != -1) {
      String ts = timestamp.substring(start + 1, end);
      return convertToEpoch(ts);
    }
  }

  SerialMon.println("Failed to parse SIM time, using RTC time");
  return rtc.getLocalEpoch();
}

// ========== Convert Timestamp to Epoch ==========
long convertToEpoch(const String &timestamp) {
  int year, month, day, hour, minute, second;
  sscanf(timestamp.c_str(), "%d/%d/%d,%d:%d:%d", &year, &month, &day, &hour, &minute, &second);
  year += 2000;

  struct tm t;
  t.tm_year = year - 1900;
  t.tm_mon = month - 1;
  t.tm_mday = day;
  t.tm_hour = hour;
  t.tm_min = minute;
  t.tm_sec = second;

  return mktime(&t);
}
