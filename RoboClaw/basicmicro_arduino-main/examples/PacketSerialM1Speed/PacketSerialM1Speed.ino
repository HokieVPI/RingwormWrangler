/**
 * Basicmicro Library Example: M1SPEED (35)
 *
 * Demonstrates setting the target speed for Motor 1 using Velocity PID control.
 * The controller will use the configured PID parameters and encoder feedback
 * to drive the motor at the commanded speed in encoder counts per second.
 * The speed is a signed 32-bit value.
 *
 * This example assumes Velocity PID parameters have been set previously
 * (e.g., using SETM1PID).
 *
 * This example uses HardwareSerial (Serial1) for communication with the controller
 * and HardwareSerial (Serial) for debugging output. Adjust serial ports and
 * controller address as needed for your setup.
 */

#include <Arduino.h>
#include <Basicmicro.h>

// Define the serial port for communication with the motor controller
// On boards like Mega, Due, Leonardo, etc., use Serial1, Serial2, Serial3.
//#define CONTROLLER_SERIAL   Serial1

// Optional: Use SoftwareSerial on AVR boards if HardwareSerial is not available
#include <SoftwareSerial.h>
#define RX_PIN 10 // Connect to controller's TX pin
#define TX_PIN 11 // Connect to controller's RX pin
SoftwareSerial controllerSerial_SW(RX_PIN, TX_PIN);
#define CONTROLLER_SERIAL   controllerSerial_SW // Use this define instead of Serial1

// Define the address of your motor controller
#define MOTOR_ADDRESS       128

// Define the library's internal read timeout in microseconds
#define LIBRARY_READ_TIMEOUT 10000

// Instantiate the Basicmicro library object
// If using SoftwareSerial, uncomment the #define above and use controllerSerial_SW
Basicmicro controller(&CONTROLLER_SERIAL, LIBRARY_READ_TIMEOUT);

// Define example target speed for Motor 1 in encoder counts per second (signed 32-bit)
int32_t motor1_target_speed = 500; // Example: 500 counts/sec forward

void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro M1SPEED Example");
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

  // Note: For Speed commands to work, Velocity PID must be enabled and tuned.
  // You might need to send SETM1PID here or ensure it's configured in the controller's NVM.
  // Example PID values (these are just placeholders, you need values tuned for your system):
  // controller.SetM1VelocityPID(MOTOR_ADDRESS, 0.1, 0.01, 0.001, 1000);

  Serial.print("Attempting to set Motor 1 target speed to: ");
  Serial.print(motor1_target_speed);
  Serial.println(" counts/sec");
}

void loop() {
  // Attempt to set the target speed for Motor 1
  // The function expects a uint32_t, so we cast our signed int32_t.
  // This results in the correct two's complement representation being sent.
  bool success = controller.SpeedM1(MOTOR_ADDRESS, (uint32_t)motor1_target_speed);

  if (success) {
    // Serial.println("M1SPEED command successful."); // Don't print every time in loop
  } else {
    Serial.println("M1SPEED command failed.");
    Serial.println("Check wiring, power, address, baud rate, and controller configuration (Is Velocity PID enabled?).");
  }

  // Change the target speed periodically
  static unsigned long lastChange = 0;
  if (millis() - lastChange > 4000) { // Change every 4 seconds
      lastChange = millis();
      if (motor1_target_speed > 0) {
          motor1_target_speed = -500; // Switch to reverse speed
      } else {
          motor1_target_speed = 500; // Switch to forward speed
      }
      Serial.print("Switching Motor 1 target speed to: ");
      Serial.print(motor1_target_speed);
      Serial.println(" counts/sec");
  }

  // Short delay to prevent flooding serial
  delay(50);
}