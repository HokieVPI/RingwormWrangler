/**
 * Basicmicro Library Example: GETM1MAXCURRENT (135)
 *
 * Demonstrates reading the configured maximum allowable current limits
 * for Motor 1. These limits control power output to protect the motor
 * and controller from excessive current draw.
 *
 * The returned limits (for driving and regenerative braking) are in milliamps (mA).
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

  Serial.println("Basicmicro GETM1MAXCURRENT Example");
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

  Serial.println("Attempting to read Motor 1 Max Current limits...");
}

void loop() {
  // Variables to store the read maximum current limits (uint32_t)
  uint32_t max_current_mA, min_current_mA;

  // Attempt to read the maximum current limits for Motor 1
  // The function returns true on success, false on failure.
  // The values are stored in max_current_mA and min_current_mA.
  bool success = controller.ReadM1MaxCurrent(MOTOR_ADDRESS, max_current_mA, min_current_mA);

  if (success) {
    Serial.println("GETM1MAXCURRENT command successful.");
    Serial.println("Motor 1 Max Current Limits (Driving/Regen):");
    Serial.print("  Max: "); Serial.print(max_current_mA); Serial.println(" 10s of mA");
    Serial.print("  Min: "); Serial.print(min_current_mA); Serial.println(" 10s of mA");
  } else {
    Serial.println("GETM1MAXCURRENT command failed.");
    Serial.println("Check wiring, power, address, and baud rate.");
  }

  // Wait a few seconds before reading again
  delay(2000); // Reading settings infrequently is fine
}