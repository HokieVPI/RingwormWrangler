/**
 * Basicmicro Library Example: GETM2ENC (17)
 *
 * Demonstrates reading the encoder count value for Motor 2 from the
 * Basicmicro motor controller. This value represents the current position
 * of Motor 2 based on encoder feedback.
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

  Serial.println("Basicmicro GETM2ENC Example");
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

  Serial.println("Attempting to read Motor 2 encoder...");
}

void loop() {
  // Variables to store the read encoder value, status, and validity flag
  uint32_t encoderValue;
  uint8_t status;
  bool valid;

  // Attempt to read the encoder value for Motor 2
  // The function returns the 32-bit encoder count.
  // The optional 'status' pointer will be filled with a status code.
  // The optional 'valid' pointer will be set to true if communication was successful.
  encoderValue = controller.ReadEncM2(MOTOR_ADDRESS, &status, &valid);

  if (valid) {
    Serial.println("GETM2ENC command successful.");
    Serial.print("Motor 2 Encoder Count: ");
    Serial.println((int32_t)encoderValue); // Cast to signed int32_t for printing
    Serial.print("Motor 2 Encoder Status: ");
    Serial.println(status);
    /*
       Status codes (based on library comments):
       0 = OK
       1 = Underflow (count minimum reached)
       2 = Overflow (count maximum reached)
       3 = Direction bit set (encoder moving in reverse direction)
    */
  } else {
    Serial.println("GETM2ENC command failed.");
    Serial.println("Check wiring, power, address, baud rate, and controller configuration (Is Motor 2 encoder enabled?).");
  }

  // Wait a short time before reading again to avoid flooding the serial
  delay(100);
}