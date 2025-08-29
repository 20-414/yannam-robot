#include <SoftwareSerial.h>

#define TXD 2   // Arduino TX pin (Connect to GSM RX)
#define RXD 3   // Arduino RX pin (Connect to GSM TX)
#define PWRKEY 4  // GSM PWRKEY pin

SoftwareSerial gsmSerial(RXD, TXD);  // RX, TX

void powerOnGSM() {
  pinMode(PWRKEY, OUTPUT);
  digitalWrite(PWRKEY, LOW);
  delay(1000);  // Hold PWRKEY low for 1 second
  digitalWrite(PWRKEY, HIGH);
  Serial.println("GSM Module Powering On...");
  delay(5000);  // Wait for the module to initialize
}

void sendATCommand(String command, int delayTime) {
  Serial.print("Sent: "); Serial.println(command);  // Print command sent
  gsmSerial.println(command);  // Send AT command
  delay(delayTime);  // Wait for response

  Serial.print("Response: ");
  while (gsmSerial.available()) {
    char c = gsmSerial.read();  // Read response character
    Serial.print(c);  // Print response
  }
  Serial.println(); // Newline after response
}

void setup() {
  Serial.begin(115200);  // Debugging with PC
  gsmSerial.begin(9600);  // GSM communication

  Serial.println("Starting Arduino GSM Communication...");
  powerOnGSM();  // Turn on the GSM module

  // Check if GSM module is responding
  delay(2000);
  sendATCommand("AT", 1000);
  
  // Check network registration
  sendATCommand("AT+CREG?", 2000);

  // Check signal strength
  sendATCommand("AT+CSQ", 2000);

  // Check APN settings
  sendATCommand("AT+CGDCONT?", 2000);

  // Attach to GPRS
  sendATCommand("AT+CGATT=1", 2000);

  // Open Network
  sendATCommand("AT+NETOPEN", 2000);
}

void loop() {
  // You can send SMS, make calls, or send data here
}
