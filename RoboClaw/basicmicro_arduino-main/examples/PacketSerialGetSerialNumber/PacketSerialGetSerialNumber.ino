/**
 * Basicmicro Library Example: GETSERIALNUMBER (97)
 *
 * Demonstrates reading the serial number string from the Basicmicro motor controller.
 * The serial number is stored as a string up to 36 characters.
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

// Buffer to store the read serial number string.
// Max 36 bytes + 1 for null terminator = 37 bytes needed.
char serialNumberBuffer[37];


void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro GETSERIALNUMBER Example");
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

  Serial.println("Attempting to read serial number...");

  // Initialize buffer to ensure it's empty if read fails
  serialNumberBuffer[0] = '\0';
}

void loop() {
  // Attempt to read the serial number string
  // The function returns true on success, false on failure.
  // The string is stored in the provided buffer.
  bool success = controller.GetSerialNumber(MOTOR_ADDRESS, serialNumberBuffer);

  if (success) {
    Serial.println("GETSERIALNUMBER command successful.");
    Serial.print("Controller Serial Number: ");
    Serial.println(serialNumberBuffer); // Print the received string
  } else {
    Serial.println("GETSERIALNUMBER command failed.");
    Serial.println("Check wiring, power, address, and baud rate.");
    serialNumberBuffer[0] = '\0'; // Ensure buffer is empty on failure
  }

  // Wait a few seconds before reading again
  delay(5000); // Reading serial number infrequently is fine
}