#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Encoder.h>

// Motor 1 (Left)
#define ENA 9
#define IN1 8
#define IN2 7

// Motor 2 (Right)
#define ENB 10
#define IN3 6
#define IN4 5

// Encoder Pins
#define ENCODER_A1 2
#define ENCODER_B1 4
#define ENCODER_A2 3
#define ENCODER_B2 12

Encoder motor1Encoder(ENCODER_A1, ENCODER_B1);
Encoder motor2Encoder(ENCODER_A2, ENCODER_B2);

ros::NodeHandle nh;

// Motor speed variables
float linear_speed = 0;
float angular_speed = 0;

void cmdVelCallback(const geometry_msgs::Twist& cmd_msg) {
  linear_speed = cmd_msg.linear.x;
  angular_speed = cmd_msg.angular.z;

  moveRobot(linear_speed, angular_speed);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmdVelCallback);

void setup() {
  nh.initNode();
  nh.subscribe(sub);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
  nh.spinOnce();
  delay(10);
}

void moveRobot(float linear, float angular) {
  int leftSpeed = (linear - angular) * 150;
  int rightSpeed = (linear + angular) * 150;

  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  controlMotor(leftSpeed, ENA, IN1, IN2);
  controlMotor(rightSpeed, ENB, IN3, IN4);
}

void controlMotor(int speed, int pwmPin, int in1, int in2) {
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwmPin, -speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, 0);
  }
}
