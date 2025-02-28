/**
 * Main ESP32 controller for Cropion Rover
 * Handles motor control, servo control, and ELRS radio receiver
 */

// Include necessary headers
#include "config.h"
#include "motors.h"
#include "servo.h"
#include "radio.h"
#include "serial_comm.h"

void setup() {
  // Initialize serial communication with main computer
  setupSerialComm();
  
  // Initialize ELRS radio receiver
  setupRadio();
  
  // Initialize motor controllers
  setupMotors();
  
  // Initialize servo
  setupServo();
  
  Serial.println("ESP32 Controller Initialized");
}

void loop() {
  // Update radio receiver data
  updateRadio();
  
  // Process commands from main computer
  processSerialCommands();
  
  // Send telemetry data to main computer
  sendTelemetry();
  
  // Small delay to prevent CPU overuse
  delay(10);
}