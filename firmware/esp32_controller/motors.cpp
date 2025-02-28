/**
 * Motor control implementation
 */

#include "motors.h"
#include "config.h"
#include <Arduino.h>

// PWM channels for motor control
#define FRONT_MOTOR_PWM_CHANNEL 0
#define REAR_MOTOR_PWM_CHANNEL 1

void setupMotors() {
  // Configure PWM for motor control
  ledcSetup(FRONT_MOTOR_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(REAR_MOTOR_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  
  // Attach PWM channels to motor pins
  ledcAttachPin(FRONT_MOTOR_PWM_PIN, FRONT_MOTOR_PWM_CHANNEL);
  ledcAttachPin(REAR_MOTOR_PWM_PIN, REAR_MOTOR_PWM_CHANNEL);
  
  // Configure direction pins
  pinMode(FRONT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(REAR_MOTOR_DIR_PIN, OUTPUT);
  
  // Initialize motors to stopped state
  stopAllMotors();
}

void setFrontMotor(int speed, int direction) {
  // Constrain speed to valid range
  speed = constrain(speed, MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);
  
  // Set direction pin
  digitalWrite(FRONT_MOTOR_DIR_PIN, direction > 0 ? HIGH : LOW);
  
  // Set speed via PWM
  ledcWrite(FRONT_MOTOR_PWM_CHANNEL, speed);
}

void setRearMotor(int speed, int direction) {
  // Constrain speed to valid range
  speed = constrain(speed, MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);
  
  // Set direction pin
  digitalWrite(REAR_MOTOR_DIR_PIN, direction > 0 ? HIGH : LOW);
  
  // Set speed via PWM
  ledcWrite(REAR_MOTOR_PWM_CHANNEL, speed);
}

void stopAllMotors() {
  // Stop both motors
  ledcWrite(FRONT_MOTOR_PWM_CHANNEL, 0);
  ledcWrite(REAR_MOTOR_PWM_CHANNEL, 0);
}