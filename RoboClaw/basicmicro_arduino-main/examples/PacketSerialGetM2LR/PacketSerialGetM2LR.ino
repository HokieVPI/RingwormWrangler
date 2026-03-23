/**
 * Basicmicro Library Example: GETM2LR (131) - MCP Only
 *
 * Demonstrates reading the configured motor inductance (L) and resistance (R)
 * parameters for Motor 2 on MCP series controllers.
 *
 * The returned parameters are floating-point values (Henries for L, Ohms for R).
 * The library converts the raw 32-bit values received from the controller back to floats.
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

  Serial.println("Basicmicro GETM2LR Example (MCP Only)");
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

  Serial.println("Attempting to read Motor 2 L/R values...");
}

void loop() {
  // Variables to store the read L and R values (floating point)
  float read_L, read_R;

  // Attempt to read the Motor 2 L and R values
  // This command is typically only supported by MCP series controllers.
  // The function returns true on success, false on failure.
  // The values are stored in read_L and read_R.
  bool success = controller.GetM2LR(MOTOR_ADDRESS, read_L, read_R);

  if (success) {
    Serial.println("GETM2LR command successful.");
    Serial.println("Motor 2 L/R Parameters:");
    Serial.print("  Inductance (L): "); Serial.print(read_L, 6); Serial.println(" H"); // Print with more precision
    Serial.print("  Resistance (R): "); Serial.print(read_R, 4); Serial.println(" Ohm"); // Print with more precision
    Serial.println("NOTE: This command is for MCP controllers.");
  } else {
    Serial.println("GETM2LR command failed.");
    Serial.println("Check wiring, power, address, baud rate, controller model, and configuration.");
    Serial.println("This command may not be supported by your controller.");
  }

  // Wait a few seconds before reading again
  delay(2000); // Reading settings infrequently is fine
}