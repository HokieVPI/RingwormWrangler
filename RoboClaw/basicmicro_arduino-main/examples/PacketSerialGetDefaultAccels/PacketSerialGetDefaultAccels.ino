/**
 * Basicmicro Library Example: GETDEFAULTACCELS (81)
 *
 * Demonstrates reading the default acceleration and deceleration rates for both
 * Motor 1 and Motor 2 from the Basicmicro motor controller. These rates are used
 * by motion commands that do not explicitly specify acceleration or deceleration.
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

  Serial.println("Basicmicro GETDEFAULTACCELS Example");
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

  Serial.println("Attempting to read default acceleration and deceleration values...");
}

void loop() {
  // Variables to store the read default acceleration and deceleration values
  uint32_t accelM1, decelM1, accelM2, decelM2;

  // Attempt to read the default acceleration/deceleration for both motors
  // The function returns true on success, false on failure.
  // The values are stored in the provided variables.
  bool success = controller.GetDefaultAccels(MOTOR_ADDRESS, accelM1, decelM1, accelM2, decelM2);

  if (success) {
    Serial.println("GETDEFAULTACCELS command successful.");
    Serial.println("Default Acceleration/Deceleration:");
    Serial.print("  Motor 1 Accel: "); Serial.print(accelM1);
    Serial.print(", Motor 1 Decel: "); Serial.println(decelM1);
    Serial.print("  Motor 2 Accel: "); Serial.print(accelM2);
    Serial.print(", Motor 2 Decel: "); Serial.println(decelM2);
    Serial.println(" (counts/sec/sec)");
  } else {
    Serial.println("GETDEFAULTACCELS command failed.");
    Serial.println("Check wiring, power, address, and baud rate.");
  }

  // Wait a few seconds before reading again
  delay(2000); // Reading settings infrequently is fine
}