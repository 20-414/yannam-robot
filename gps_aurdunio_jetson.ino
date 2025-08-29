#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

// GPS pins
#define RXPin 3  // GPS TX → D3
#define TXPin 4  // GPS RX → D4
#define GPSBaud 9600

// Geofence center and radius (in meters)
#define GEOFENCE_LAT 17.4271
#define GEOFENCE_LON 78.4321
#define GEOFENCE_RADIUS 70

TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);

// Calculate distance using Haversine formula
double distance(double lat1, double lon1, double lat2, double lon2) {
  double R = 6371000; // Earth radius (m)
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(radians(lat1)) * cos(radians(lat2)) *
             sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

void setup() {
  Serial.begin(115200);       // Jetson serial
  gpsSerial.begin(GPSBaud);   // GPS serial
}

void loop() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isUpdated()) {
    double lat = gps.location.lat();
    double lon = gps.location.lng();
    double alt = gps.altitude.meters();
    int sats = gps.satellites.value();

    // Calculate distance from geofence center
    double dist = distance(lat, lon, GEOFENCE_LAT, GEOFENCE_LON);
    String status = (dist > GEOFENCE_RADIUS) ? "outside" : "inside";

    // Time from GPS (convert UTC to IST)
    int hour = gps.time.hour();
    int minute = gps.time.minute() + 30;
    int second = gps.time.second();
    if (minute >= 60) {
      minute -= 60;
      hour += 1;
    }
    hour += 5;
    if (hour >= 24) hour -= 24;

    char timeBuffer[9];
    sprintf(timeBuffer, "%02d:%02d:%02d", hour, minute, second);

    // Create JSON string
    String json = "{";
    json += "\"lat\": " + String(lat, 6) + ",";
    json += "\"lon\": " + String(lon, 6) + ",";
    json += "\"alt\": " + String(alt, 2) + ",";
    json += "\"sats\": " + String(sats) + ",";
    json += "\"time\": \"" + String(timeBuffer) + "\",";
    json += "\"dist\": " + String(dist, 2) + ",";
    json += "\"status\": \"" + status + "\"";
    json += "}";

    Serial.println(json);  // Send to Jetson
    delay(1000);           // Adjust as needed
  }
}
