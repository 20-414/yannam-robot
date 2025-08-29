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
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme; // I2C
String getTimeFromSIMModule();
//void WriteJsonToEEPROM();
//void ReadandPublishJsonFromEEPROM();
//void resetEEPROM();

#define RXPin 16    // VVM501 MODULE RXD INTERNALLY CONNECTED
#define TXPin 17    // VVM501 MODULE TXD INTERNALLY CONNECTED
#define powerPin 4  // VVM501 MODULE ESP32 PIN D4 CONNECTED TO POWER PIN OF A7670C CHIPSET, INTERNALLY CONNECTED
#define RXD2 27    //VVM501 MODULE RXD INTERNALLY CONNECTED
#define TXD2 26    //VVM501 MODULE TXD INTERNALLY CONNECTED

//#define PACKET_SIZE 200 // Define the size of each packet
//#define EEPROM_START_ADDR 0 // Start address in EEPROM to store packets

RTC_DATA_ATTR int numStoredPackets = 0; // Variable to store the number of packets stored

HardwareSerial GPSSerial(2); // Use hardware serial port 1

ESP32Time rtc(3600);  // offset in seconds GMT+1
// const char *broker = "broker.emqx.io"; // MQTT Broker address
// const char *messageTopic = "sensor_data"; // MQTT topic to publish sensor data
// const char *ClientID = "mqttx_87eba806";
// const char *Username = "emqx1";
// const char *Password = "public";

TinyGPSPlus gps;
int num=0;
int rx = -1;
String rxString;
int _timeout;
String _buffer;
String jsonPayload;
char jsonStr[512]; // Declare jsonStr globally
uint8_t mac[6];
String epochTime;
RTC_DATA_ATTR int bootCount = 0;

bool storeInEEprom = false;
const int sleepTimeSeconds = 90;// deep sleep time

#define EEPROM_SIZE 4096  // Adjust the size as needed
#define MAX_PACKET_SIZE 200
#define EEPROM_PREF_INDEX (EEPROM_SIZE - 2) // Index for storing preference value

const char apn[] = ""; // APN automatically detects for 4G SIM, NO NEED TO ENTER, KEEP IT BLANK

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm        modem(debugger);
#else
TinyGsm        modem(SerialAT);
#endif
TinyGsmClient client(modem);
PubSubClient  mqtt(client);

void setup() {
  Serial.begin(115200);
 //   while (!Serial);
 // Serial.println(F("BME680 async test"));
 // GPSSerial.begin(9600, SERIAL_8N1, RXPin, TXPin);
 // pinMode(powerPin, OUTPUT);
 // digitalWrite(powerPin, LOW);
 // delay(100);
 // digitalWrite(powerPin, HIGH);
 // delay(1000);
 // digitalWrite(powerPin, LOW);

 // if (!bme.begin(0x76)) {
   // Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
 // }
  // Set up oversampling and filter initialization
 // bme.setTemperatureOversampling(BME680_OS_8X);
 // bme.setHumidityOversampling(BME680_OS_2X);
 // bme.setPressureOversampling(BME680_OS_4X);
 // bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
 // bme.setGasHeater(320, 150); // 320*C for 150 ms

  
   // Initialize EEPROM
 // if (!EEPROM.begin(EEPROM_SIZE)) {
   // Serial.println("Failed to initialize EEPROM");
   // while (true); // Hang here if EEPROM initialization fails
  //}
  
  // Read the stored preference value from EEPROM
  //storeInEEprom = EEPROM.read(EEPROM_SIZE - 1); // Read the last byte of EEPROM where the preference is stored
  //Serial.print("pref:");
  //Serial.println(storeInEEprom);

     // Get ESP32 MAC Address
  //esp_read_mac(mac, ESP_MAC_WIFI_STA);

  // Print MAC address to serial monitor
  //Serial.print("MAC Address: ");
  //for (int i = 0; i < 6; i++) {
    //Serial.printf("%02x", mac[i]);
    //if (i < 5) {
      //Serial.print(":");
   // }
 // }
 // Serial.println();

//Increment boot number and print it every reboot
//  ++bootCount;
 // Serial.println("Boot number: " + String(bootCount));

   
 // Serial.println("\nconfiguring VVM501 Module. Kindly wait");
 // delay(3000);

SerialAT.begin(115200, SERIAL_8N1, RXD2, TXD2);
  DBG("Initializing modem...");
  if (!modem.init()) {
    DBG("Failed to restart modem, delaying 10s and retrying");
    return;   
  }
  DBG("Initializing modem...");
  if (!modem.restart()) {
    Serial.println("Failed to restart modem,delaying 10s and retrying");
    return;
  }
 //  String name = modem.getModemName();
 // DBG("Modem Name:", name);

 // String modemInfo = modem.getModemInfo();
 // DBG("Modem Info:", modemInfo);
  
  Serial.println("Waiting for network...");
  if (!modem.waitForNetwork()) {
    Serial.println("Network initialization failed.");
     delay(5000);
    return;
  }
  Serial.println("success");

   if (modem.isNetworkConnected()) {
    Serial.println("Network connected");
  }

  // GPRS connection parameters are usually set after network registration
  Serial.print(F("Connecting to "));
  Serial.print(apn);
  if (!modem.gprsConnect(apn)) {
    Serial.println("GPRS connection failed.");
    delay(5000);
    return;
  }
  Serial.println("success");

  if (modem.isGprsConnected()) {
    Serial.println("LTE module connected");
  }

  // intialize the gps module
  Serial.println("GPS module initialised");

  // Call getTimeFromSIMModule() to get the epoch time
String epochTimeString = getTimeFromSIMModule();
long epochTime = epochTimeString.toInt(); // Convert String to long

Serial.print("Epoch time from SIM Module: ");
Serial.println(epochTime);

rtc.setTime(epochTime); // Set RTC time with epoch time
 
  // MQTT Broker setup
 // mqtt.setServer(broker, 1883);
 // mqtt.setCallback(callback); 
}

void loop() {
 // if (!mqtt.connected()) {
   // reconnect(); // Reconnect to MQTT if necessary
 // }

 if (!modem.isNetworkConnected()) {
    Serial.println("Network disconnected");
    // Store data in EEPROM if network is disconnected
    //run_process(); // Pass sensor data to storing in EEPROM
    //WriteJsonToEEPROM(jsonStr);// fun to store data in eeprom
   // WriteJsonToEEPROM();
   // Serial.print("pref:");//bool to know the data stored
   // Serial.println(storeInEEprom);
   // delay(100);
   // Serial.printf("Entering deep sleep for %d seconds...\n", sleepTimeSeconds);
   // ESP.deepSleep(sleepTimeSeconds * 1000000); // Convert seconds to microseconds
   }
  else{
   // run_process();//Pass sensor data to the mqtt 
   // mqtt.publish(messageTopic, jsonStr);//convert the srt to char
    Serial.println("Sensor data published to mqtt");
    //Serial.println(jsonStr);
  }
 
 // if(storeInEEprom == true){
   // Serial.println("entering into storein pref");
   //readAndPublishPacketFromPreferences();
   // ReadandPublishJsonFromEEPROM();
   // Serial.println("data packet retrived from eeprom");
   
  }
  
  // Enter deep sleep mode for the specified time
 // Serial.println("end of loop");
 // Serial.printf("Entering deep sleep for %d seconds...\n", sleepTimeSeconds);
  //ESP.deepSleep(sleepTimeSeconds * 1000000); // Convert seconds to microseconds
//}

// function to run the process
//void run_process() {
  // Read sensor data
// float latitude, longitude;
  //  delay(1000);
  //while (GPSSerial.available() > 0) {
    //if (gps.encode(GPSSerial.read())) {
      //if (gps.location.isValid()) {
        //Serial.print("Latitude: ");
       // Serial.println(gps.location.lat(), 6);
       // Serial.print("Longitude: ");
       // Serial.println(gps.location.lng(), 6);
       // Serial.println("gps success");
     // } else {
       // Serial.println("GPS data invalid");
      //}
   // }
 // }

  //  float temperature, humidity, pressure, altitude, gas_resistance; // Declare sensor data variables
   // Tell BME680 to begin measurement.
 // unsigned long endTime = bme.beginReading();
 // if (endTime == 0) {
  //  Serial.println(F("Failed to begin reading :("));
  //  return;
//  }

//if (!bme.endReading()) {
  //  Serial.println(F("Failed to complete reading :("));
    //return;
//  }


  // Assign sensor data values to variables
 // temperature = bme.temperature;
 // humidity = bme.humidity;
 // pressure = bme.pressure / 100.0; // Convert to hPa
 // altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
 // gas_resistance = bme.gas_resistance / 1000.0; // Convert to KOhms

  
 //Serial.print(F("Temperature = "));
 // Serial.print(bme.temperature);
 // Serial.println(F(" *C"));
  
 // Serial.print(F("Humidity = "));
 // Serial.print(bme.humidity);
 // Serial.println(F(" %"));

//  Serial.print(F("Pressure = "));
//  Serial.print(bme.pressure / 100.0);
 // Serial.println(F(" hPa"));

 // Serial.print(F("Gas = "));
 // Serial.print(bme.gas_resistance / 1000.0);
 // Serial.println(F(" KOhms"));

 // Serial.print(F("Approx. Altitude = "));
 // Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
 // Serial.println(F(" m"));

 // latitude = gps.location.lat();
 // longitude = gps.location.lng();

 // Convert sensor data to strings for MQTT
 // String tempStr = String(temperature);
 // String humiStr = String(humidity);
 // String presStr = String(pressure);
 // String altStr = String(altitude);
 // String gasStr = String(gas_resistance);
 // String latStr = String(latitude, 6);
 // String lonStr = String(longitude, 6);

  
  // Generate MAC address string
 // char macStr[10];
 // snprintf(macStr, sizeof(macStr), "ZDL1%02x%02x", mac[4], mac[5]);
 // String Uid = macStr;
  
  // Get epoch time from SIM module
 // String epochTime = getTimeFromSIMModule();
 // String Epoch = epochTime;
 

// Create a JSON object for sensor data
 // DynamicJsonDocument jsonBuffer(512);
 // JsonObject root = jsonBuffer.to<JsonObject>();
 // root["temp"] = tempStr;
 // root["humi"] = humiStr;
 // root["pres"] = presStr;
 // root["alt"] = altStr;
 // root["gas"] = gasStr;
 // root["lat"] = latStr;
 // root["long"] = lonStr;
  // Update the JSON log with MAC address
 // root["Uid"] = macStr;
 // root["Epoch"] = epochTime; // Add SIM module timestamp to JSON log
  // Serialize the JSON object to a string
 // serializeJson(root, jsonStr);
 // Serial.println(jsonStr);
    
 // mqtt.loop();
 // Serial.println("end of run process fun");
//}

// Function definition
//float altitude(const int32_t press, const float seaLevel) {
 // return 44330.0 * (1.0 - pow(((float)press / 100.0) / seaLevel, 0.1903)); 
//}

//void WriteJsonToEEPROM() {
  // Write the JSON string dataToStore to EEPROM
 // Serial.println("Writing data to EEPROM...");

 // int jsonLength = strlen(jsonStr);
 // Serial.print("jsonLength:");
 // Serial.println(jsonLength);

  // Check if there's enough space in EEPROM
 // if ((numStoredPackets + 1) * PACKET_SIZE > (EEPROM_SIZE - 1)) {
   // Serial.println("Error: Not enough space in EEPROM to store packets.");
   // return;
 // }

  // Calculate the start address for this packet
 // int startAddress = EEPROM_START_ADDR + numStoredPackets * PACKET_SIZE;
 // Serial.print("startAddress of packet: ");
 // Serial.println(startAddress);
  // Write packet size as the first byte
 // EEPROM.write(startAddress, min(PACKET_SIZE, jsonLength));
 // startAddress++;

  // Write the JSON string to EEPROM
  //for (int i = 0; i < jsonLength; i++) {
    //EEPROM.write(startAddress + i, jsonStr[i]);
 // }

  // Update the number of stored packets
  //numStoredPackets++;

  // Save the number of stored packets to EEPROM
  //EEPROM.write(EEPROM_PREF_INDEX, numStoredPackets);

  // Save changes to EEPROM
  //EEPROM.commit();
  //storeInEEprom = true;

 // Serial.println("Data written to EEPROM");
 // Serial.print("Number of packets stored:");
 // Serial.println(numStoredPackets);

  // Store the preference value in EEPROM before going to sleep
 // EEPROM.write(EEPROM_SIZE - 1, storeInEEprom); // Write the preference value to the last byte of EEPROM
 // EEPROM.commit(); // Save changes to EEPROM
 // delay(100);
 // Serial.println("end of writejson to eeprom fun");
//}


//void ReadandPublishJsonFromEEPROM() {
 // Serial.println("Reading data from EEPROM...");

  // Read the number of stored packets from EEPROM
 // numStoredPackets = EEPROM.read(EEPROM_PREF_INDEX);
 // Serial.print("Number of packets stored: ");
 // Serial.println(numStoredPackets);

  // Read each packet from EEPROM and print it
 // for (int packetNum = 0; packetNum < numStoredPackets; packetNum++) {
   // int startAddress = EEPROM_START_ADDR + packetNum * PACKET_SIZE;
   // int packetSize = EEPROM.read(startAddress);
   // startAddress++;

     // Construct the complete packet
   // String packet;
   // for (int i = 0; i < packetSize; i++) {
     // char dataChar = EEPROM.read(startAddress + i);
     // packet += dataChar;
   // }

       // Publish each character to MQTT
      // mqtt.publish(messageTopic, packet.c_str());
     
   //  Serial.print("Packet ");
   // Serial.print(packetNum);
   // Serial.print(" (Size ");
   // Serial.print(packetSize);
   // Serial.print("): ");
   // Serial.println(packet);
   // }
   // Update the EEPROM preference value
   // storeInEEprom = false;
   // EEPROM.write(EEPROM_SIZE - 1, storeInEEprom);
   // EEPROM.commit(); // Save changes to EEPROM
   // delay(100);

     // Reset EEPROM if needed
  //resetEEPROM();
 // numStoredPackets=0;

 // Serial.println("End of reading data from EEPROM");
//}

String convertTimestampToEpoch(const String &timestamp) {
  // Parse the timestamp string to extract year, month, day, hour, minute, and second
  int year, month, day, hour, minute, second;
  sscanf(timestamp.c_str(), "%d/%d/%d,%d:%d:%d", &year, &month, &day, &hour, &minute, &second);
  
  // Adjust for the year
  year += 2000;

  // Create a tm structure to convert to epoch time
  struct tm tm;
  tm.tm_year = year - 1900;
  tm.tm_mon = month - 1;  // Month is 0-based
  tm.tm_mday = day;
  tm.tm_hour = hour;
  tm.tm_min = minute;
  tm.tm_sec = second;

  // Convert to epoch time
  time_t epochTime = mktime(&tm);

  
epochTime;
}

String getTimeFromSIMModule() {
  String timestamp;

  // Send AT command to retrieve timestamp from SIM module
  SerialAT.println("AT+CCLK?");
  delay(100);

  // Read response from SIM module
  while (SerialAT.available()) {
    char c = SerialAT.read();
    timestamp += c;
  }

  // Print the raw timestamp received
  Serial.println("Raw timestamp from SIM Module: " + timestamp);

  // Process 'timestamp' string to extract the time information
  int timestampIndex = timestamp.indexOf("+CCLK: ");
  if (timestampIndex != -1) {
    timestamp.remove(0, timestampIndex + 7); // Remove "+CCLK: " from the string
    timestamp.trim(); // Remove leading/trailing spaces
    int timestampStart = timestamp.indexOf('\"'); // Find the start of the actual timestamp
    int timestampEnd = timestamp.lastIndexOf('\"'); // Find the end of the timestamp

    // Extract time information from the remaining string
    // Example: timestamp format is "YY/MM/DD,HH:MM:SS±TZ"
    // Extracting time from the string based on the format of your SIM module's response
    if (timestampStart != -1 && timestampEnd != -1) {
      // Extract the timestamp
      timestamp = timestamp.substring(timestampStart + 1, timestampEnd);

      // Print the extracted timestamp
//      Serial.println("Extracted timestamp: " + timestamp);

      // Convert timestamp to epoch time
      return String(convertTimestampToEpoch(timestamp));
    }
  }

  // If timestamp extraction fails, return local RTC time
  return String(rtc.getLocalEpoch()); // return local RTC time if SIM module time not available
}




//void reconnect() {
//    unsigned long startAttemptTime = millis();

 //   while (!mqtt.connected()) {
   //     Serial.print("Attempting MQTT connection...");

     //   boolean status = mqtt.connect("4g modem test");

       // if (status == false) {
         //   Serial.println(" fail");
           // Serial.print("failed, rc=");
           // Serial.print(mqtt.state());
          //  Serial.println(" try again in 5 seconds");

            // Check if the timeout period has elapsed
          //  if (millis() - startAttemptTime > MQTT_CONNECT_TIMEOUT) {
            //    Serial.println("MQTT connection attempt timed out");
              //  break;  // Exit the loop if timeout is reached
           // }

          //  delay(5000);
     //   } else {
       //     Serial.println(" success");
       // }
   // }
//}

//void resetEEPROM() {
    // Reset the EEPROM data
  //  for (int i = 0; i < EEPROM_SIZE; ++i) {
    //    EEPROM.write(i, 0);
   // }
   // EEPROM.commit();
   // num = 0;
//    eepromData = false;

   // Serial.println("EEPROM data reset");
//}


void callback(char* topic, byte* payload, unsigned int length) {
  // Handle MQTT callback if needed
}
