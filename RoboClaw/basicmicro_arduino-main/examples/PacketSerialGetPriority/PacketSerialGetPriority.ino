/**
 * Basicmicro Library Example: GETPRIORITY (140) - MCP Only
 *
 * Demonstrates reading the configured priority levels for different control
 * modes (e.g., Serial, RC, Analog) on MCP series controllers. Higher priority
 * modes will override lower priority modes if multiple inputs are active.
 *
 * The specific number of priority levels and their meaning depend on the
 * specific MCP model and its configuration.
 *
 * !!! IMPORTANT: This command is marked as "MCP only". It may not work on
 * Sabertooth or Kangaroo controllers. Consult your controller's documentation
 * regarding the meaning of priority values for each level. !!!
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

  Serial.println("Basicmicro GETPRIORITY Example (MCP Only)");
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

  Serial.println("Attempting to read priority levels...");
  Serial.println("NOTE: This command is for MCP controllers.");
}

void loop() {
  // Variables to store the read priority levels (unsigned 8-bit)
  // Assuming 3 priority levels based on the SETPRIORITY function signature.
  uint8_t p1, p2, p3;

  // Attempt to read the priority levels
  // This command is typically only supported by MCP series controllers.
  // The function returns true on success, false on failure.
  // The values are stored in p1, p2, and p3.
  bool success = controller.GetPriority(MOTOR_ADDRESS, p1, p2, p3);

  if (success) {
    Serial.println("GETPRIORITY command successful.");
    Serial.println("Current Priority Levels:");
    Serial.print("  Priority 1: "); Serial.println(p1);
    Serial.print("  Priority 2: "); Serial.println(p2);
    Serial.print("  Priority 3: "); Serial.println(p3);
    Serial.println("NOTE: Lower number often means higher priority. Consult documentation.");
  } else {
    Serial.println("GETPRIORITY command failed.");
    Serial.println("Check wiring, power, address, baud rate, and controller model.");
    Serial.println("This command may not be supported by your controller.");
  }

  // Wait a few seconds before reading again
  delay(3000); // Reading settings infrequently is fine
}