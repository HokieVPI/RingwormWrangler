/**
 * Basicmicro Library Example: SETM2ENCCOUNT (23)
 *
 * Demonstrates setting the encoder count value for Motor 2 to a specific value.
 * This is useful for calibrating position or setting a known home position.
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

// Define the target encoder count to set for Motor 2
// Use int32_t for the value as encoders can be signed.
int32_t targetEncoderCount = -500; // Example: Set encoder to -500 counts

void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro SETM2ENCCOUNT Example");
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

  Serial.print("Attempting to set Motor 2 encoder count to: ");
  Serial.println(targetEncoderCount);
}

void loop() {
  // Attempt to set the encoder count for Motor 2
  // The function returns true on success, false on failure.
  bool success = controller.SetEncM2(MOTOR_ADDRESS, targetEncoderCount);

  if (success) {
    Serial.println("SETM2ENCCOUNT command successful.");
    // Optionally, read back the encoder value to verify
    uint32_t currentEnc;
    bool valid;
    currentEnc = controller.ReadEncM2(MOTOR_ADDRESS, NULL, &valid);
    if (valid) {
        Serial.print("Verified Motor 2 Encoder Count: ");
        Serial.println((int32_t)currentEnc);
    } else {
        Serial.println("Failed to read Motor 2 encoder for verification.");
    }
  } else {
    Serial.println("SETM2ENCCOUNT command failed.");
    Serial.println("Check wiring, power, address, baud rate, and controller configuration.");
  }

  // This command is typically not sent repeatedly in a loop.
  // For this example, we'll add a long delay.
  delay(10000); // Wait 10 seconds before attempting again
}