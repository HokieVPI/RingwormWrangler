/**
 * Basicmicro Library Example: GETSCRIPTAUTORUN (247) - MCP Only
 *
 * Demonstrates reading the configured script autorun time on MCP series
 * controllers that support onboard scripting. This indicates the delay in
 * milliseconds after startup before the script begins execution.
 *
 * A returned value of 0 typically means script autorun is disabled.
 *
 * !!! IMPORTANT: This command is marked as "MCP only" and requires scripting support.
 * It may not work on Sabertooth or Kangaroo controllers or MCPs without scripting.
 * Consult your controller's documentation. !!!
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

  Serial.println("Basicmicro GETSCRIPTAUTORUN Example (MCP Only)");
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

  Serial.println("Attempting to read Script Autorun Time...");
  Serial.println("NOTE: This command is for MCP controllers with scripting support.");
}

void loop() {
  // Variable to store the read script autorun time (uint32_t)
  uint32_t read_autorun_time_ms;

  // Attempt to read the script autorun time
  // This command is typically only supported by MCP series controllers with scripting.
  // The function returns true on success, false on failure.
  // The value is stored in read_autorun_time_ms.
  bool success = controller.GetScriptAutorun(MOTOR_ADDRESS, read_autorun_time_ms);

  if (success) {
    Serial.println("GETSCRIPTAUTORUN command successful.");
    Serial.print("Current Script Autorun Time: ");
    Serial.print(read_autorun_time_ms); Serial.println(" ms"); // 0 means disabled
    if (read_autorun_time_ms == 0) {
        Serial.println("  (Script autorun is disabled)");
    }
  } else {
    Serial.println("GETSCRIPTAUTORUN command failed.");
    Serial.println("Check wiring, power, address, baud rate, and controller model.");
    Serial.println("This command may not be supported by your controller or scripting is not enabled.");
  }

  // Wait a few seconds before reading again
  delay(3000); // Reading settings infrequently is fine
}