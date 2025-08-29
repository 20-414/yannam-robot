#include <TinyGPS++.h>
#include <TinyGPSPlus.h>

#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// Define GPS module connections
#define RXPin 3  // GPS TX → Arduino D3
#define TXPin 4  // GPS RX → Arduino D4
#define GPSBaud 9600

// Initialize software serial and TinyGPS++
SoftwareSerial gpsSerial(RXPin, TXPin);
TinyGPSPlus gps;

void setup() {
    Serial.begin(115200);  // Open serial monitor at 115200 baud
    gpsSerial.begin(GPSBaud);
    Serial.println("GPS Module Test - Waiting for GPS signal...");
}

void loop() {
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
        
        if (gps.location.isUpdated()) {  
            Serial.print("Latitude: "); Serial.println(gps.location.lat(), 6);
            Serial.print("Longitude: "); Serial.println(gps.location.lng(), 6);
            Serial.print("Altitude: "); Serial.print(gps.altitude.meters()); Serial.println(" m");
            Serial.print("Satellites: "); Serial.println(gps.satellites.value());
            Serial.print("Time: "); Serial.print(gps.time.hour()); Serial.print(":");
            Serial.print(gps.time.minute()); Serial.print(":");
            Serial.println(gps.time.second());

            Serial.println("--------------------------------");
        }
    }
}
