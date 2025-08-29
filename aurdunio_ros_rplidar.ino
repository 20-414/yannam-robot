#include <Encoder.h> // Library for motor encoders

// Motor 1 (Left)
#define ENA 9  // PWM pin
#define IN1 8
#define IN2 7

// Motor 2 (Right)
#define ENB 10 // PWM pin
#define IN3 6
#define IN4 5

// Encoder pins for Motor 1 (Left)
#define ENCODER_A1 2  // Must be an interrupt pin
#define ENCODER_B1 4

// Encoder pins for Motor 2 (Right)
#define ENCODER_A2 3  // Must be an interrupt pin
#define ENCODER_B2 12

// Create Encoder objects
Encoder motor1Encoder(ENCODER_A1, ENCODER_B1);
Encoder motor2Encoder(ENCODER_A2, ENCODER_B2);

long positionMotor1 = 0;
long positionMotor2 = 0;

void setup() {
  Serial.begin(9600); // Start serial communication

  // Set motor control pins as output
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.println("Arduino Ready. Waiting for commands...");
}

void loop() {
  // Read movement command from Jetson Nano
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    if (command == 'f') moveMotorForward();
    else if (command == 'b') moveMotorBackward();
    else if (command == 'l') turnLeft();
    else if (command == 'r') turnRight();
    else stopMotors();
  }

  // Read Encoder values
  long newPositionMotor1 = motor1Encoder.read();
  long newPositionMotor2 = motor2Encoder.read();

  if (newPositionMotor1 != positionMotor1 || newPositionMotor2 != positionMotor2) {
    positionMotor1 = newPositionMotor1;
    positionMotor2 = newPositionMotor2;

    Serial.print("Left Motor Encoder: ");
    Serial.println(positionMotor1);
    Serial.print("Right Motor Encoder: ");
    Serial.println(positionMotor2);
  }

  delay(100);
}

// Function to move forward
void moveMotorForward() {
  Serial.println("Moving Forward");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 200); // Adjust speed

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 200);
}

// Function to move backward
void moveMotorBackward() {
  Serial.println("Moving Backward");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 200);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 200);
}

// Function to turn left
void turnLeft() {
  Serial.println("Turning Left");
  digitalWrite(IN1, LOW);  // Left motor backward
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 100);  // Slow speed

  digitalWrite(IN3, HIGH); // Right motor forward
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 255);  // Fast speed
  delay(500); // Adjust turn duration
  stopMotors();
}

// Function to turn right
void turnRight() {
  Serial.println("Turning Right");
  digitalWrite(IN1, HIGH); // Left motor forward
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 255);  // Fast speed

  digitalWrite(IN3, LOW);  // Right motor backward
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 100);  // Slow speed
  delay(500); // Adjust turn duration
  stopMotors();
}

// Function to stop motors
void stopMotors() {
  Serial.println("Stopping Motors");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}
