/**
 * Serial communication interface for ESP32 Controller
 */

#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

// Setup serial communication
void setupSerialComm();

// Process commands received from the main computer
void processSerialCommands();

// Send telemetry data to the main computer
void sendTelemetry();

#endif // SERIAL_COMM_H