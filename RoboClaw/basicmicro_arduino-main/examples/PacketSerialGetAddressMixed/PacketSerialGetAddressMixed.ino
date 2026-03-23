/**
 * Basicmicro Library Example: GETADDRESSMIXED (142) - MCP Only
 *
 * Demonstrates reading the current controller address and the state of
 * addressed mixing mode on MCP series controllers.
 *
 * !!! IMPORTANT: This command is marked as "MCP only". It may not work on
 * Sabertooth or Kangaroo controllers. Consult your controller's documentation
 * regarding addressed mixing. !!!
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

  Serial.println("Basicmicro GETADDRESSMIXED Example (MCP Only)");
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

  Serial.println("Attempting to read controller address and mixing mode...");
  Serial.println("NOTE: This command is for MCP controllers.");
}

void loop() {
  // Variables to store the read address and mixing enabled flag
  uint8_t read_address, read_mixing_enabled;

  // Attempt to read the address and mixing state
  // This command is typically only supported by MCP series controllers.
  // The function returns true on success, false on failure.
  // The values are stored in read_address and read_mixing_enabled.
  bool success = controller.GetAddressMixed(MOTOR_ADDRESS, read_address, read_mixing_enabled);

  if (success) {
    Serial.println("GETADDRESSMIXED command successful.");
    Serial.println("Controller Settings:");
    Serial.print("  Current Address: "); Serial.println(read_address);
    Serial.print("  Addressing Mixing Enabled: "); Serial.println(read_mixing_enabled); // 0=Disabled, 1=Enabled
  } else {
    Serial.println("GETADDRESSMIXED command failed.");
    Serial.println("Check wiring, power, address, baud rate, and controller model.");
    Serial.println("This command may not be supported by your controller.");
  }

  // Wait a few seconds before reading again
  delay(3000); // Reading settings infrequently is fine
}