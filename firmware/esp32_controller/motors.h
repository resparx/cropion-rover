/**
 * Motor control functions for ESP32 Controller
 */

#ifndef MOTORS_H
#define MOTORS_H

// Setup motor controllers
void setupMotors();

// Set front motor speed and direction
void setFrontMotor(int speed, int direction);

// Set rear motor speed and direction
void setRearMotor(int speed, int direction);

// Stop all motors
void stopAllMotors();

#endif // MOTORS_H