/**
 * Basicmicro Library Example: GETDOUTS (138) - MCP Only
 *
 * Demonstrates reading the configured actions for all digital output pins
 * on MCP series controllers. This command returns the total number of digital
 * outputs and the action value configured for each.
 *
 * !!! IMPORTANT: This command is marked as "MCP only". It may not work on
 * Sabertooth or Kangaroo controllers. Consult your controller's documentation
 * regarding the meaning of the action values. !!!
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

// Buffer to store the digital output actions.
// Choose a size large enough for the maximum number of DOUTs your controller has.
// The library header doesn't specify a max, but 8 is a common max for digital outputs.
#define MAX_DOUTS 8
uint8_t digitalOutputActions[MAX_DOUTS];


void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro GETDOUTS Example (MCP Only)");
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
  delay(100); // Short delay to let the controller initialize after power-up or serial init

  Serial.println("Attempting to read digital output configurations...");
  Serial.println("NOTE: This command is for MCP controllers.");
}

void loop() {
  // Variable to store the number of digital outputs reported by the controller
  uint8_t dout_count = 0; // Initialize to 0

  // Attempt to read the digital output actions
  // The function returns true on success, false on failure.
  // The number of outputs is stored in dout_count, and the actions
  // are stored in the digitalOutputActions array (up to MAX_DOUTS).
  bool success = controller.GetDOUTS(MOTOR_ADDRESS, dout_count, digitalOutputActions, MAX_DOUTS);

  if (success) {
    Serial.println("GETDOUTS command successful.");
    Serial.print("Reported Digital Output Count: "); Serial.println(dout_count);
    Serial.println("Configured Digital Output Actions:");

    // Iterate up to the reported count (or MAX_DOUTS if reported count is larger)
    uint8_t display_count = min(dout_count, MAX_DOUTS);
    for (uint8_t i = 0; i < display_count; i++) {
        Serial.print("  Output "); Serial.print(i);
        Serial.print(" Action: "); Serial.println(digitalOutputActions[i]);
    }
    if (dout_count > MAX_DOUTS) {
        Serial.print("(Only displaying first "); Serial.print(MAX_DOUTS); Serial.println(" outputs due to buffer size)");
    }
    Serial.println("NOTE: The meaning of action values is specific to your controller model.");

  } else {
    Serial.println("GETDOUTS command failed.");
    Serial.println("Check wiring, power, address, baud rate, and controller model.");
    Serial.println("This command may not be supported by your controller.");
  }

  // Wait a few seconds before reading again
  delay(3000); // Reading settings infrequently is fine
}