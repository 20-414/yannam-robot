#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

// GPS module pins
#define RXPin 3  // GPS TX → Arduino D3
#define TXPin 4  // GPS RX → Arduino D4
#define GPSBaud 9600

// Initialize TinyGPS++ and SoftwareSerial
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);

// Geofence center and radius
#define GEOFENCE_LAT 17.4271
#define GEOFENCE_LON 78.4321
#define GEOFENCE_RADIUS 70  // meters

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(GPSBaud);
  Serial.println("GPS + Geofencing - Initializing...");
}

void loop() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());

    if (gps.location.isUpdated()) {
      double lat = gps.location.lat();
      double lon = gps.location.lng();
      double alt = gps.altitude.meters();
      int sats = gps.satellites.value();
      double dist = distance(lat, lon, GEOFENCE_LAT, GEOFENCE_LON);

      // Get UTC time from GPS
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

      // Print GPS data
      Serial.print("Latitude: "); Serial.println(lat, 6);
      Serial.print("Longitude: "); Serial.println(lon, 6);
      Serial.print("Altitude: "); Serial.print(alt); Serial.println(" m");
      Serial.print("Satellites: "); Serial.println(sats);

      Serial.print("IST Time: ");
      if (hour < 10) Serial.print('0');
      Serial.print(hour); Serial.print(":");
      if (minute < 10) Serial.print('0');
      Serial.print(minute); Serial.print(":");
      if (second < 10) Serial.print('0');
      Serial.println(second);

      Serial.print("Distance from center: ");
      Serial.print(dist, 2);
      Serial.print(" m — ");

      if (dist >= GEOFENCE_RADIUS) {
        Serial.println("Outside Geofence");
      } else {
        Serial.println("Inside Geofence");
      }

      Serial.println("--------------------------------");
    }
  }
}

// Haversine distance function in meters
double distance(double lat1, double lon1, double lat2, double lon2) {
  double R = 6371000; // Earth radius in meters
  double phi1 = radians(lat1);
  double phi2 = radians(lat2);
  double deltaPhi = radians(lat2 - lat1);
  double deltaLambda = radians(lon2 - lon1);

  double a = sin(deltaPhi / 2) * sin(deltaPhi / 2) +
             cos(phi1) * cos(phi2) *
             sin(deltaLambda / 2) * sin(deltaLambda / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return R * c;
}
