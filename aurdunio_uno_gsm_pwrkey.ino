#include <SoftwareSerial.h>
#define PWRKEY  4   // Connect A7670C PWRKEY to Arduino pin 4
#define STATUS  5   // Optional: Connect STATUS pin to Arduino pin 5
SoftwareSerial A7670Serial(2, 3);  // RX, TX
void powerOnModule() {
  pinMode(PWRKEY, OUTPUT);
  digitalWrite(PWRKEY, LOW);
  delay(3000);  // Hold PWRKEY LOW for 3 seconds
  digitalWrite(PWRKEY, HIGH);
  Serial.println("Powering on A7670C...");
  delay(8000);  // Increased boot time
}
bool checkModuleStatus() {
  pinMode(STATUS, INPUT);
  return digitalRead(STATUS);  // Returns 1 if ON, 0 if OFF
}
void sendATCommand(const char* cmd, const char* expectedResponse, unsigned long timeout) {
  Serial.print("Sending command: ");
  Serial.println(cmd);
  A7670Serial.println(cmd);
  delay(200);
  String response = "";
  unsigned long startTime = millis();
  bool responseOK = false;
  while (millis() - startTime < timeout) {
    while (A7670Serial.available() > 0) {
      char c = A7670Serial.read();
      response += c;
    }
    if (response.indexOf(expectedResponse) != -1) {
      responseOK = true;
      break;
    }
  }
  Serial.print("Response: ");
  Serial.println(response);
  if (responseOK)
    Serial.println(":white_check_mark: Response OK");
  else
    Serial.println(":hourglass_flowing_sand: Timeout: No expected response");
}
void setup() {
  Serial.begin(9600);
  A7670Serial.begin(9600);  // Use 9600 baud for stability
  powerOnModule();  // Turn on A7670C module automatically
  delay(2000);
  // Check if module is ON using STATUS pin
  if (checkModuleStatus()) {
    Serial.println(":white_check_mark: A7670C is ON");
  } else {
    Serial.println(":warning: A7670C is OFF, check power or press PWRKEY manually.");
  }
  // Directly test communication before using the function
  Serial.println("Testing AT command...");
  A7670Serial.println("AT");
  delay(1000);
  while (A7670Serial.available()) {
    Serial.write(A7670Serial.read());
  }
  Serial.println();
  delay(2000);
  sendATCommand("AT", "OK", 3000);  // Check communication
}
void loop() {
  // Serial pass-through (allows manual AT commands)
  if (Serial.available()) {
    A7670Serial.write(Serial.read());
  }
  if (A7670Serial.available()) {
    Serial.write(A7670Serial.read());
  }
}
