#include <HardwareSerial.h>

HardwareSerial sim800(2); // Using UART2 (GPIO16 RX, GPIO17 TX)

void setup() {
    Serial.begin(115200);
    sim800.begin(9600, SERIAL_8N1, 26, 27); // Change baud rate if needed
    delay(3000);
    Serial.println("Testing GSM Module...");
    
    // Send AT command with carriage return & newline
    Serial.println("Sending AT command...");
    sim800.print("AT\r\n");
}

void loop() {
    if (sim800.available()) {
        Serial.print("GSM Response: ");
        while (sim800.available()) {
            Serial.write(sim800.read()); // Print GSM response
        }
        Serial.println();
    } else {
        Serial.println("No response from GSM module.");
    }
    delay(2000);
}
