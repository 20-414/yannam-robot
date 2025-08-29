#include <SoftwareSerial.h>

SoftwareSerial rplidarSerial(10, 11); // RX, TX (Use appropriate pins for your board)

void setup() {
  Serial.begin(115200);            // For communication with the computer
  rplidarSerial.begin(115200);     // For communication with RPLIDAR (use RPLIDAR's baud rate)
}

void loop() {
  if (rplidarSerial.available()) {
    char data = rplidarSerial.read(); // Read data from RPLIDAR
    Serial.write(data);              // Forward data to the computer
  }
}
