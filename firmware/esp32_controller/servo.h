/**
 * Servo control functions for ESP32 Controller
 */

#ifndef SERVO_H
#define SERVO_H

// Setup servo
void setupServo();

// Set servo angle (0-180 degrees)
void setServoAngle(int angle);

// Center the servo (90 degrees)
void centerServo();

#endif // SERVO_H