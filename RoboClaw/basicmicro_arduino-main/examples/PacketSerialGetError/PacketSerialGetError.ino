/**
 * Basicmicro Library Example: GETERROR (90)
 *
 * Demonstrates reading the 32-bit error and warning status flags from the
 * Basicmicro motor controller. Each bit in the returned value represents a
 * specific error or warning condition (e.g., E-Stop, battery low, overtemperature).
 *
 * You can use the error code definitions from Basicmicro.h to decode the status.
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

  Serial.println("Basicmicro GETERROR Example");
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

  Serial.println("Attempting to read error and warning flags...");
}

void loop() {
  // Variable to store the 32-bit error/warning status value
  uint32_t status_flags;
  bool valid;

  // Attempt to read the error status
  // The function returns the 32-bit status value.
  // The optional 'valid' pointer will be set to true if communication was successful.
  status_flags = controller.ReadError(MOTOR_ADDRESS, &valid);

  if (valid) {
    Serial.println("GETERROR command successful.");
    Serial.print("Controller Status Flags: 0x");
    Serial.println(status_flags, HEX); // Print as hexadecimal

    // Decode individual flags based on definitions in Basicmicro.h
    Serial.println("Detected Conditions:");
    if (status_flags == 0) Serial.println("  No errors or warnings.");

    // Errors (typically latching)
    if (status_flags & controller.ERROR_ESTOP)      Serial.println("  ERROR: E-Stop active");
    if (status_flags & controller.ERROR_TEMP)       Serial.println("  ERROR: Temperature Sensor 1 >=100C");
    if (status_flags & controller.ERROR_TEMP2)      Serial.println("  ERROR: Temperature Sensor 2 >=100C");
    if (status_flags & controller.ERROR_LBATHIGH)   Serial.println("  ERROR: Logic Battery High Voltage");
    if (status_flags & controller.ERROR_LBATLOW)    Serial.println("  ERROR: Logic Battery Low Voltage");
    if (status_flags & controller.ERROR_SPEED1)     Serial.println("  ERROR: Motor 1 Speed Error Limit");
    if (status_flags & controller.ERROR_SPEED2)     Serial.println("  ERROR: Motor 2 Speed Error Limit");
    if (status_flags & controller.ERROR_POS1)       Serial.println("  ERROR: Motor 1 Position Error Limit");
    if (status_flags & controller.ERROR_POS2)       Serial.println("  ERROR: Motor 2 Position Error Limit");
    if (status_flags & controller.ERROR_CURRENTM1)  Serial.println("  ERROR: Motor 1 Current Limited");
    if (status_flags & controller.ERROR_CURRENTM2)  Serial.println("  ERROR: Motor 2 Current Limited");

    // Warnings (typically transient)
    if (status_flags & controller.WARN_OVERCURRENTM1) Serial.println("  WARN: Motor 1 Current Limited");
    if (status_flags & controller.WARN_OVERCURRENTM2) Serial.println("  WARN: Motor 2 Current Limited");
    if (status_flags & controller.WARN_MBATHIGH)    Serial.println("  WARN: Main Battery Voltage High");
    if (status_flags & controller.WARN_MBATLOW)     Serial.println("  WARN: Main Battery Low Voltage");
    if (status_flags & controller.WARN_TEMP)        Serial.println("  WARN: Temperaure Sensor 1 >=85C");
    if (status_flags & controller.WARN_TEMP2)       Serial.println("  WARN: Temperature Sensor 2 >=85C");
    if (status_flags & controller.WARN_S4)          Serial.println("  WARN: Motor 1 Home/Limit Signal");
    if (status_flags & controller.WARN_S5)          Serial.println("  WARN: Motor 2 Home/Limit Signal");
    if (status_flags & controller.WARN_BOOT)        Serial.println("  WARN: Unit is booting"); // Set on power up, cleared when fully booted
    if (status_flags & controller.WARN_OVERREGENM1) Serial.println("  WARN: Motor 1 Regen Current Limited");
    if (status_flags & controller.WARN_OVERREGENM2) Serial.println("  WARN: Motor 2 Regen Current Limited");


  } else {
    Serial.println("GETERROR command failed.");
    Serial.println("Check wiring, power, address, and baud rate.");
  }

  // Wait a few seconds before reading again
  delay(1000); // Reading status periodically is common
}