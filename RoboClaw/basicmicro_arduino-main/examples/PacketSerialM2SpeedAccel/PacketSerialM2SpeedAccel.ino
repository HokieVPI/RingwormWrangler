/**
 * Basicmicro Library Example: M2SPEEDACCEL (39)
 *
 * Demonstrates setting the target speed for Motor 2 with controlled acceleration
 * using Velocity PID control. The motor will accelerate at the specified rate
 * (in counts/sec/sec) until it reaches the target speed.
 *
 * This command requires Velocity PID to be enabled and tuned for Motor 2.
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

// Define example acceleration and target speed for Motor 2
uint32_t motor2_accel = 250; // Acceleration rate in counts/sec/sec
int32_t motor2_target_speed = -900; // Target speed in counts/sec (signed)

void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro M2SPEEDACCEL Example");
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
  // You might need to send SETM2PID here or ensure it's configured in the controller's NVM.
  // Example PID values (these are just placeholders, you need values tuned for your system):
  // controller.SetM2VelocityPID(MOTOR_ADDRESS, 0.08, 0.008, 0.0008, 1200);

  Serial.print("Attempting to set Motor 2 accel: "); Serial.print(motor2_accel);
  Serial.print(" and target speed: "); Serial.print(motor2_target_speed);
  Serial.println(" counts/sec");
}

void loop() {
  // Attempt to set the target speed for Motor 2 with acceleration
  // The speed parameter expects a uint32_t, so we cast our signed int32_t.
  bool success = controller.SpeedAccelM2(MOTOR_ADDRESS, motor2_accel, (uint32_t)motor2_target_speed);

  if (success) {
    // Serial.println("M2SPEEDACCEL command successful."); // Don't print every time in loop
  } else {
    Serial.println("M2SPEEDACCEL command failed.");
    Serial.println("Check wiring, power, address, baud rate, and controller configuration (Is Velocity PID enabled?).");
  }

  // Change the target speed periodically
  static unsigned long lastChange = 0;
  if (millis() - lastChange > 5000) { // Change every 5 seconds
      lastChange = millis();
      if (motor2_target_speed < 0) {
          motor2_target_speed = 900; // Switch to forward speed
      } else {
          motor2_target_speed = -900; // Switch to reverse speed
      }
      Serial.print("Switching Motor 2 target speed to: ");
      Serial.print(motor2_target_speed);
      Serial.println(" counts/sec");
  }

  // Short delay to prevent flooding serial
  delay(50);
}