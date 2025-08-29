#include <Encoder.h>

// motor 1 settings (Left Motor)
#define ENA 9
#define IN1 8
#define IN2 7

// motor 2 settings (Right Motor)
#define ENB 10
#define IN3 6
#define IN4 5

// Encoder pins for motor 1 (Left Motor)
#define ENCODER_A1 2
#define ENCODER_B1 4

// Encoder pins for motor 2 (Right Motor)
#define ENCODER_A2 3
#define ENCODER_B2 12

Encoder motor1Encoder(ENCODER_A1, ENCODER_B1);
Encoder motor2Encoder(ENCODER_A2, ENCODER_B2);

long positionMotor1 = 0;
long positionMotor2 = 0;

void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
  // Check if a command is received from Jetson Nano
  if (Serial.available() > 0) {
    char command = Serial.read();

    if (command == 'f') {
      moveMotorForward();
    } else if (command == 'b') {
      moveMotorBackward();
    } else if (command == 's') {
      stopMotors();
    } else if (command == 'l') {
      turnLeft();
    } else if (command == 'r') {
      turnRight();
    }
  }

  // Send encoder values back to the Jetson Nano
  long newPositionMotor1 = motor1Encoder.read();
  long newPositionMotor2 = motor2Encoder.read();

  if (newPositionMotor1 != positionMotor1 || newPositionMotor2 != positionMotor2) {
    positionMotor1 = newPositionMotor1;
    positionMotor2 = newPositionMotor2;

    // Send encoder values to Jetson Nano in CSV format
    Serial.print(positionMotor1);
    Serial.print(",");
    Serial.println(positionMotor2);
  }

  delay(100);
}

void moveMotorForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 200);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 200);
}

void moveMotorBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 200);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 200);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 100);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 255);
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 255);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 100);
}
