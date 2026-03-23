/**
 * Basicmicro Library Example: GETVERSION (21)
 *
 * Demonstrates reading the firmware version string from the Basicmicro motor controller.
 * This can be useful for identifying the controller model and capabilities.
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

// Buffer to store the version string (max 48 bytes + null terminator)
char versionString[49]; // Library doc says max 48 bytes including null, use 49 just to be safe

void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro GETVERSION Example");
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

  Serial.println("Attempting to read controller version...");

  // Attempt to read the version string
  // The function returns true on success, false on failure.
  // The version string is stored in the provided buffer.
  bool success = controller.ReadVersion(MOTOR_ADDRESS, versionString);

  if (success) {
    Serial.println("GETVERSION command successful.");
    Serial.print("Controller Version: ");
    Serial.println(versionString);
  } else {
    Serial.println("GETVERSION command failed.");
    Serial.println("Check wiring, power, address, and baud rate.");
    versionString[0] = '\0'; // Ensure buffer is empty on failure
  }
}

void loop() {
  // This command is usually called once in setup or infrequently.
  // The loop can be empty, or you can add a long delay.
  // For this example, we'll just delay to keep it running.
  delay(5000); // Wait 5 seconds before checking (though version doesn't change)
}