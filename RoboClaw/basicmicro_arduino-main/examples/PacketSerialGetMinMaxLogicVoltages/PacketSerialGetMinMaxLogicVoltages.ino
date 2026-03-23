/**
 * Basicmicro Library Example: GETMINMAXLOGICVOLTAGES (60)
 *
 * Demonstrates reading the configured minimum and maximum voltage limits
 * for the logic battery (controller power supply) from the Basicmicro motor controller.
 *
 * The returned voltage values are in tenths of a volt.
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

  Serial.println("Basicmicro GETMINMAXLOGICVOLTAGES Example");
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

  Serial.println("Attempting to read Logic Battery voltage limits...");
}

void loop() {
  // Variables to store the read voltage limits (in tenths of a volt)
  uint16_t minVoltage_tenths, maxVoltage_tenths;

  // Attempt to read the logic battery voltage limits
  // The function returns true on success, false on failure.
  // The values are stored in the provided variables.
  bool success = controller.ReadMinMaxLogicVoltages(MOTOR_ADDRESS, minVoltage_tenths, maxVoltage_tenths);

  if (success) {
    Serial.println("GETMINMAXLOGICVOLTAGES command successful.");
    Serial.println("Logic Battery Voltage Limits:");
    Serial.print("  Minimum: "); Serial.print((float)minVoltage_tenths / 10.0, 1); Serial.println(" V");
    Serial.print("  Maximum: "); Serial.print((float)maxVoltage_tenths / 10.0, 1); Serial.println(" V");
  } else {
    Serial.println("GETMINMAXLOGICVOLTAGES command failed.");
    Serial.println("Check wiring, power, address, and baud rate.");
  }

  // Wait a few seconds before reading again
  delay(2000);
}