/**
 * Basicmicro Library Example: SETM2PID (29)
 *
 * Demonstrates setting the Velocity PID control parameters for Motor 2.
 * PID control is used for closed-loop speed control based on encoder feedback.
 *
 * This example uses HardwareSerial (Serial1) for communication with the controller
 * and HardwareSerial (Serial) for debugging output. Adjust serial ports and
 * controller address as needed for your setup.
 */

#include <Arduino.h>
#include <Basicmicro.h>

// Define the serial port for communication with the motor controller
// On boards like Mega, Due, Leonardo, etc., use Serial1, Serial2, Serial3.
#define CONTROLLER_SERIAL   Serial1

// Optional: Use SoftwareSerial on AVR boards if HardwareSerial is not available
// #include <SoftwareSerial.h>
// #define RX_PIN 10 // Connect to controller's TX pin
// #define TX_PIN 11 // Connect to controller's RX pin
// SoftwareSerial controllerSerial_SW(RX_PIN, TX_PIN);
// #define CONTROLLER_SERIAL   controllerSerial_SW // Use this define instead of Serial1

// Define the address of your motor controller
#define MOTOR_ADDRESS       128

// Define the library's internal read timeout in microseconds
#define LIBRARY_READ_TIMEOUT 10000

// Instantiate the Basicmicro library object
// If using SoftwareSerial, uncomment the #define above and use controllerSerial_SW
Basicmicro controller(&CONTROLLER_SERIAL, LIBRARY_READ_TIMEOUT);

// Define example PID parameters for Motor 2 Velocity control
// These values will need tuning for your specific motor and encoder setup.
float m2_Kp = 0.08;    // Proportional gain
float m2_Ki = 0.008;   // Integral gain
float m2_Kd = 0.0008;  // Derivative gain
// Define the max velocity in quadrature pulses per second (QPPS)
// This should typically match the max speed the motor/encoder can achieve,
// as it's used for scaling PID calculations.
uint32_t m2_MaxQpps = 1200; // Example: Assume max speed is 1200 QPPS

void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro SETM2PID Example");
  Serial.print("Connecting to controller on ");
  // Print the name of the serial port being used (if possible)
  #if defined(CONTROLLER_SERIAL) && !defined(RX_PIN) // Check if using HardwareSerial and not SoftwareSerial
    Serial.println("Hardware Serial");
  #elif defined(RX_PIN) // Check if SoftwareSerial pins are defined
    Serial.println("Software Serial");
  #else
    Serial.println("Unknown Serial type");
  #endif


  // Initialize the communication serial port for the controller
  controller.begin(9600); // Ensure baud rate matches your controller
  delay(100); // Short delay to let the controller initialize after power-up or serial init

  Serial.println("Attempting to set Motor 2 Velocity PID parameters...");
  Serial.print("Kp: "); Serial.print(m2_Kp);
  Serial.print(", Ki: "); Serial.print(m2_Ki);
  Serial.print(", Kd: "); Serial.print(m2_Kd);
  Serial.print(", MaxQPPS: "); Serial.println(m2_MaxQpps);
}

void loop() {
  // Attempt to set the Velocity PID parameters for Motor 2
  // The function handles converting the float PID values to the controller's fixed-point format.
  // It returns true on success, false on failure.
  bool success = controller.SetM2VelocityPID(MOTOR_ADDRESS, m2_Kp, m2_Ki, m2_Kd, m2_MaxQpps);

  if (success) {
    Serial.println("SETM2PID command successful.");
    // You might want to verify the settings by reading them back
    float read_Kp, read_Ki, read_Kd;
    uint32_t read_qpps;
    if (controller.ReadM2VelocityPID(MOTOR_ADDRESS, read_Kp, read_Ki, read_Kd, read_qpps)) {
        Serial.println("Verified Motor 2 Velocity PID:");
        Serial.print("  Kp: "); Serial.print(read_Kp);
        Serial.print(", Ki: "); Serial.print(read_Ki);
        Serial.print(", Kd: "); Serial.print(read_Kd);
        Serial.print(", MaxQPPS: "); Serial.println(read_qpps);
    } else {
        Serial.println("Failed to read Motor 2 Velocity PID for verification.");
    }
  } else {
    Serial.println("SETM2PID command failed.");
    Serial.println("Check wiring, power, address, baud rate, and controller configuration.");
  }

  // PID settings are typically set once in setup or on configuration changes.
  // For this example, we'll add a long delay.
  delay(10000); // Wait 10 seconds before attempting again
}