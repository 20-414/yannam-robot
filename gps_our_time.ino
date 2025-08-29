#include <TinyGPS++.h>
#include <SoftwareSerial.h>

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

            // Get UTC time
            int hour = gps.time.hour();
            int minute = gps.time.minute();
            int second = gps.time.second();

            // Convert to IST (UTC +5:30)
            minute += 30;
            if (minute >= 60) {
                minute -= 60;
                hour += 1;
            }
            hour += 5;
            if (hour >= 24) {
                hour -= 24;
            }

            Serial.print("IST Time: ");
            if (hour < 10) Serial.print('0');
            Serial.print(hour); Serial.print(":");
            if (minute < 10) Serial.print('0');
            Serial.print(minute); Serial.print(":");
            if (second < 10) Serial.print('0');
            Serial.println(second);

            Serial.println("--------------------------------");
        }
    }
}
