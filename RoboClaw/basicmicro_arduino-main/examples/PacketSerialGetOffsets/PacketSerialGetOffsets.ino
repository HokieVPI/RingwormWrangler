/**
 * Basicmicro Library Example: GETOFFSETS (116) - MCP Only
 *
 * Demonstrates reading the configured offset values for both encoders on
 * MCP series controllers. These offsets are typically used to fine-tune
 * encoder readings.
 *
 * !!! IMPORTANT: This command is marked as "MCP only". It may not work on
 * Sabertooth or Kangaroo controllers. Consult your controller's documentation. !!!
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

  Serial.println("Basicmicro GETOFFSETS Example (MCP Only)");
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

  Serial.println("Attempting to read encoder offsets...");
}

void loop() {
  // Variables to store the read encoder offsets (unsigned 8-bit, 0-255)
  uint8_t offset1, offset2;

  // Attempt to read the encoder offset values
  // This command is typically only supported by MCP series controllers.
  // The function returns true on success, false on failure.
  // The values are stored in offset1 and offset2.
  bool success = controller.GetOffsets(MOTOR_ADDRESS, offset1, offset2);

  if (success) {
    Serial.println("GETOFFSETS command successful.");
    Serial.println("Current Encoder Offsets:");
    Serial.print("  Encoder 1 Offset: "); Serial.println(offset1);
    Serial.print("  Encoder 2 Offset: "); Serial.println(offset2);
    Serial.println("NOTE: The meaning of these values is specific to your controller model (MCP only).");
  } else {
    Serial.println("GETOFFSETS command failed.");
    Serial.println("Check wiring, power, address, baud rate, controller model, and configuration.");
    Serial.println("This command may not be supported by your controller.");
  }

  // Wait a few seconds before reading again
  delay(2000); // Reading settings infrequently is fine
}