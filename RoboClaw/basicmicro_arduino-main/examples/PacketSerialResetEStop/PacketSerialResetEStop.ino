/**
 * Basicmicro Library Example: RESETESTOP (200) - MCP Only
 *
 * Demonstrates resetting an emergency stop (E-Stop) condition on MCP series
 * controllers. This command is effective only if the controller's E-Stop
 * lock state is configured to allow software reset (e.g., using SETESTOPLOCK).
 *
 * !!! IMPORTANT: This command is marked as "MCP only". It may not work on
 * Sabertooth or Kangaroo controllers. Consult your controller's documentation
 * regarding E-Stop functionality and lock states. !!!
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


void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro RESETESTOP Example (MCP Only)");
  Serial.print("Connecting to controller on ");
  // Print the name of the serial port being used (if possible)
  #if defined(CONTROLLER_SERIAL) && !defined(RX_PIN) // Check if using HardwareSerial and not SoftwareSerial
    Serial.println("Hardware Serial");
  #elif defined(RX_PIN) // Check if SoftwareSerial pins are defined using RX_PIN
    Serial.println("Software Serial");
  #else
    Serial.println("Unknown Serial type");
  #endif


  // Initialize the communication serial port for the controller
  controller.begin(9600); // Ensure baud rate matches your controller
  delay(100); // Short delay

  Serial.println("Attempting to reset E-Stop...");
  Serial.println("NOTE: This command is for MCP controllers and requires software reset to be enabled.");
  Serial.println("Ensure the E-Stop is active before sending this command to see its effect.");

  // You might first need to trigger an E-Stop (e.g., via a digital input)
  // or ensure it's currently active before testing the reset.
  // You may also need to set the E-Stop lock state using SETESTOPLOCK (201)
  // to allow software reset (lockState = 0xAA).
  // controller.SetEStopLock(MOTOR_ADDRESS, 0xAA);
  // delay(100);
}

void loop() {
  // Attempt to reset the E-Stop condition
  // This command is typically only supported by MCP series controllers.
  // The function returns true on success, false on failure.
  bool success = controller.ResetEStop(MOTOR_ADDRESS);

  if (success) {
    Serial.println("RESETESTOP command successful.");
    Serial.println("E-Stop condition should now be cleared (if software reset is allowed).");
  } else {
    Serial.println("RESETESTOP command failed.");
    Serial.println("Check wiring, power, address, baud rate, controller model, and configuration (Is software reset allowed?).");
    Serial.println("This command may not be supported by your controller.");
  }

  // This command should only be executed when needed, not repeatedly in a loop.
  // The loop will pause indefinitely after the first attempt.
  while(true); // Stop execution after setup and the single command attempt
}