#include <Encoder.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

// Motor Pins
#define ENA 9
#define IN1 8
#define IN2 7
#define ENB 10
#define IN3 6
#define IN4 5

// Encoder Pins
#define ENCODER_A1 2
#define ENCODER_B1 4
#define ENCODER_A2 3
#define ENCODER_B2 12

// GPS Pins (change if needed)
#define GPS_RX 3  // GPS TX → D3 (conflict with ENCODER_A2)
#define GPS_TX 4  // GPS RX → D4 (conflict with ENCODER_B1)
#define GPSBaud 9600

// Geofence Parameters
#define GEOFENCE_LAT 17.4271
#define GEOFENCE_LON 78.4321
#define GEOFENCE_RADIUS 70

Encoder motor1Encoder(ENCODER_A1, ENCODER_B1);
Encoder motor2Encoder(ENCODER_A2, ENCODER_B2);
TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX, GPS_TX); // Use software serial for GPS

unsigned long lastGPSTime = 0;

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);     // Communication with Jetson
  gpsSerial.begin(GPSBaud); // GPS module serial

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

// ---------- Main Loop ----------
void loop() {
  // Handle motor commands from Jetson
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 'f') moveForward();
    else if (command == 'b') moveBackward();
    else if (command == 's') stopMotors();
    else if (command == 'l') turnLeft();
    else if (command == 'r') turnRight();
  }

  // Send encoder feedback to Jetson
  Serial.print("ENCODER L:");
  Serial.print(motor1Encoder.read());
  Serial.print(" R:");
  Serial.println(motor2Encoder.read());

  // Read GPS data
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  // Send GPS data every 1 second
  if (gps.location.isUpdated() && millis() - lastGPSTime > 1000) {
    lastGPSTime = millis();
    sendGPSData();
  }

  delay(100);
}

// ---------- Motor Control Functions ----------
void moveForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, 200);
  analogWrite(ENB, 200);
}

void moveBackward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, 200);
  analogWrite(ENB, 200);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void turnLeft() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, 100);
  analogWrite(ENB, 255);
}

void turnRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, 255);
  analogWrite(ENB, 100);
}

// ---------- GPS + Geofence Handling ----------
double distance(double lat1, double lon1, double lat2, double lon2) {
  double R = 6371000;
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(radians(lat1)) * cos(radians(lat2)) *
             sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

void sendGPSData() {
  double lat = gps.location.lat();
  double lon = gps.location.lng();
  double alt = gps.altitude.meters();
  int sats = gps.satellites.value();

  double dist = distance(lat, lon, GEOFENCE_LAT, GEOFENCE_LON);
  String status = (dist > GEOFENCE_RADIUS) ? "outside" : "inside";

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

  String json = "{";
  json += "\"lat\": " + String(lat, 6) + ",";
  json += "\"lon\": " + String(lon, 6) + ",";
  json += "\"alt\": " + String(alt, 2) + ",";
  json += "\"sats\": " + String(sats) + ",";
  json += "\"time\": \"" + String(timeBuffer) + "\",";
  json += "\"dist\": " + String(dist, 2) + ",";
  json += "\"status\": \"" + status + "\"";
  json += "}";

  Serial.println(json);
}
