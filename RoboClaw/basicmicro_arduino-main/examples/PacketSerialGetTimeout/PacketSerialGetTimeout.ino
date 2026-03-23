/**
 * Basicmicro Library Example: GETTIMEOUT (15)
 *
 * Demonstrates reading the communication timeout value from the Basicmicro motor controller.
 * This value determines how long the controller waits for a command before entering failsafe.
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

// Define the library's internal read timeout in microseconds (e.g., 100000 us = 100 ms)
#define LIBRARY_READ_TIMEOUT 10000

// Instantiate the Basicmicro library object
// If using SoftwareSerial, uncomment the #define above and use controllerSerial_SW
Basicmicro controller(&CONTROLLER_SERIAL, LIBRARY_READ_TIMEOUT);


void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro GETTIMEOUT Example");
  Serial.print("Connecting to controller on ");
  // Print the name of the serial port being used (if possible)
  #if defined(CONTROLLER_SERIAL) && !defined(RX_PIN) // Check if using HardwareSerial and not SoftwareSerial
    Serial.println("Hardware Serial");
  #elif defined(RX_PIN) // Check if SoftwareSerial pins are defined
    Serial.println("Software Serial");
  #else
    Serial.println("Unknown Serial type");
  #endif


  // Initialize the communication serial port for the controller
  controller.begin(9600); // Ensure baud rate matches your controller
  delay(100); // Short delay to let the controller initialize after power-up or serial init

  Serial.println("Attempting to read controller timeout...");
}

void loop() {
  // Variable to store the read timeout value in seconds
  float readTimeoutSeconds = 0.0;

  // Attempt to read the controller's communication timeout
  // The function returns true on success, false on failure.
  // The timeout value is stored in the readTimeoutSeconds variable.
  bool success = controller.GetTimeout(MOTOR_ADDRESS, readTimeoutSeconds);

  if (success) {
    Serial.println("GETTIMEOUT command successful.");
    Serial.print("Controller timeout is: ");
    Serial.print(readTimeoutSeconds);
    Serial.println(" seconds.");
  } else {
    Serial.println("GETTIMEOUT command failed.");
    Serial.println("Check wiring, power, address, and baud rate.");
  }

  // Wait a few seconds before reading again
  delay(5000);
}