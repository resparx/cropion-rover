/**
 * Configuration for ESP32 Controller
 * Defines pin assignments and other constants
 */

#ifndef CONFIG_H
#define CONFIG_H

// Serial configuration
#define COMPUTER_SERIAL Serial
#define COMPUTER_BAUD_RATE 115200

// ELRS/CRSF configuration
#define CRSF_SERIAL Serial1
#define CRSF_BAUD_RATE 420000
#define CRSF_RX_PIN 16  // ESP32 pin connected to CRSF TX
#define CRSF_TX_PIN 17  // ESP32 pin connected to CRSF RX

// Motor pins
#define FRONT_MOTOR_PWM_PIN 25
#define FRONT_MOTOR_DIR_PIN 26
#define REAR_MOTOR_PWM_PIN 27
#define REAR_MOTOR_DIR_PIN 14

// Servo pin
#define SERVO_PIN 13

// Radio channel assignments
#define THROTTLE_CHANNEL 0  // Index of throttle channel in CRSF
#define STEERING_CHANNEL 1  // Index of steering channel in CRSF
#define MODE_CHANNEL 2      // Index of mode selection channel
#define ARM_CHANNEL 4       // Index of arm/disarm channel

// PWM configuration
#define PWM_FREQUENCY 5000      // 5 kHz
#define PWM_RESOLUTION 8        // 8-bit resolution (0-255)
#define SERVO_PULSE_MIN 500     // Minimum pulse width in microseconds
#define SERVO_PULSE_MAX 2500    // Maximum pulse width in microseconds

// Motor configuration
#define MOTOR_MIN_SPEED 0
#define MOTOR_MAX_SPEED 255

// Servo configuration
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define SERVO_CENTER_ANGLE 90

// Control parameters
#define THROTTLE_DEADBAND 50    // Deadband for throttle channel
#define STEERING_DEADBAND 50    // Deadband for steering channel

// Telemetry configuration
#define TELEMETRY_INTERVAL 100  // Interval in ms to send telemetry

#endif // CONFIG_H