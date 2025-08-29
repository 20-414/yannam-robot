#include <Arduino.h>

// Function to send AT commands and check response
void sendATCommand(const char* cmd, const char* expectedResponse, unsigned long timeout) {
  Serial.print("Sending command: ");
  Serial.println(cmd);
  Serial2.println(cmd);  // Send AT command to GSM module
  delay(500);

  String response = "";
  unsigned long startTime = millis();
  bool responseOK = false;

  while (millis() - startTime < timeout) {
    while (Serial2.available() > 0) {
      char c = Serial2.read();
      response += c;
      delay(10);
    }
    if (response.indexOf(expectedResponse) != -1) {
      responseOK = true;
      break;
    }
  }

  Serial.print("Response: ");
  Serial.println(response);
  if (responseOK)
    Serial.println("✅ Response OK");
  else
    Serial.println("⏳ Timeout: No expected response");
}

void setup() {
  Serial.begin(115200);  // USB Serial for debugging
  Serial2.begin(115200); // GSM module communication (internally connected to ESP32)

  delay(5000);  // Allow GSM module to initialize

  Serial.println("Checking GSM module...");

  // Send AT command to verify communication
  sendATCommand("AT", "OK", 3000);  

  // Check SIM Card Registration
  sendATCommand("AT+CPIN?", "READY", 3000);  

  // Check Network Registration
  sendATCommand("AT+CREG?", "0,1", 5000);  

  // Check Signal Strength
  sendATCommand("AT+CSQ", "+CSQ", 3000);  

  // Check Operator Name
  sendATCommand("AT+COPS?", "+COPS", 5000);  

  Serial.println("GSM Module Initialization Complete.");
}

void loop() {
  // Serial pass-through for debugging (Type AT commands manually in Serial Monitor)
  if (Serial.available()) {
    Serial2.write(Serial.read());
  }
  if (Serial2.available()) {
    Serial.write(Serial2.read());
  }
}
