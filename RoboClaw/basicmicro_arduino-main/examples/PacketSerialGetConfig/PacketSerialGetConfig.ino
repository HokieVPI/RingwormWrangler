/**
 * Basicmicro Library Example: GETCONFIG (99)
 *
 * Demonstrates reading the current 16-bit configuration value from the
 * Basicmicro motor controller. This value is a bit field containing various
 * settings that affect the controller's general behavior and features.
 *
 * !!! IMPORTANT: The meaning of each bit in the 16-bit configuration value is
 * controller-model-specific. You MUST consult your specific controller's
 * documentation to understand what each bit controls. !!!
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

  Serial.println("Basicmicro GETCONFIG Example");
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

  Serial.println("Attempting to read configuration value...");
}

void loop() {
  // Variable to store the read 16-bit configuration value
  uint16_t read_config_value;

  // Attempt to read the configuration value
  // The function returns true on success, false on failure.
  // The value is stored in read_config_value.
  bool success = controller.GetConfig(MOTOR_ADDRESS, read_config_value);

  if (success) {
    Serial.println("GETCONFIG command successful.");
    Serial.print("Current Configuration Value: 0x");
    Serial.println(read_config_value, HEX); // Print as hexadecimal
    Serial.print("Decimal: ");
    Serial.println(read_config_value);
    Serial.println("NOTE: The meaning of this value is specific to your controller model.");
  } else {
    Serial.println("GETCONFIG command failed.");
    Serial.println("Check wiring, power, address, and baud rate.");
  }

  // Wait a few seconds before reading again
  delay(5000); // Reading settings infrequently is fine
}