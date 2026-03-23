/**
 * Basicmicro Library Example: GETESTOPLOCK (202) - MCP Only
 *
 * Demonstrates reading the configured emergency stop (E-Stop) lock state
 * on MCP series controllers. This indicates how an active E-Stop condition
 * can be cleared.
 *
 * Returned lock states:
 * 0x00 = Hardware reset required
 * 0x55 = Automatic reset
 * 0xAA = Software reset allowed
 *
 * !!! IMPORTANT: This command is marked as "MCP only". It may not work on
 * Sabertooth or Kangaroo controllers. Consult your controller's documentation
 * regarding E-Stop functionality. !!!
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

  Serial.println("Basicmicro GETESTOPLOCK Example (MCP Only)");
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

  Serial.println("Attempting to read E-Stop Lock State...");
  Serial.println("NOTE: This command is for MCP controllers.");
}

void loop() {
  // Variable to store the read E-Stop lock state (uint8_t)
  uint8_t lock_state;

  // Attempt to read the E-Stop lock state
  // This command is typically only supported by MCP series controllers.
  // The function returns true on success, false on failure.
  // The state is stored in lock_state.
  bool success = controller.GetEStopLock(MOTOR_ADDRESS, lock_state);

  if (success) {
    Serial.println("GETESTOPLOCK command successful.");
    Serial.print("Current E-Stop Lock State: 0x"); Serial.print(lock_state, HEX);
    Serial.print(" (");
    switch(lock_state) {
        case 0x00: Serial.print("Hardware Reset Required"); break;
        case 0x55: Serial.print("Automatic Reset"); break;
        case 0xAA: Serial.print("Software Reset Allowed"); break;
        default: Serial.print("Unknown State"); break;
    }
    Serial.println(")");
  } else {
    Serial.println("GETESTOPLOCK command failed.");
    Serial.println("Check wiring, power, address, baud rate, and controller model.");
    Serial.println("This command may not be supported by your controller.");
  }

  // Wait a few seconds before reading again
  delay(3000); // Reading settings infrequently is fine
}