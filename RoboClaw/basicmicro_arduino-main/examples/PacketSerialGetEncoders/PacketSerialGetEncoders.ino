/**
 * Basicmicro Library Example: GETENCODERS (78)
 *
 * Demonstrates reading the current encoder count values for both Motor 1
 * and Motor 2 in a single command. These values represent the current
 * position of each motor based on encoder feedback.
 *
 * This example uses HardwareSerial (Serial1) for communication with the controller
 * and HardwareSerial (Serial) for debugging output. Adjust serial ports and
 * controller address as needed for your setup.
 */

#include <Arduino.h>
#include <Basicmicro.h>

// Define the serial port for communication with the mc:\Users\acidtech\OneDrive\Code\Basicmicro Libraries\Basicmicro_arduino\examples\PacketSerialGetISpeeds\PacketSerialGetISpeeds.inootor controller
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

  Serial.println("Basicmicro GETENCODERS Example");
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

  Serial.println("Attempting to read encoder values for both motors...");

  // Optionally reset encoders to start from zero for easier tracking
  // Serial.println("Resetting encoders...");
  // controller.ResetEncoders(MOTOR_ADDRESS);
  // delay(100);
}

void loop() {
  // Variables to store the read encoder values (uint32_t)
  uint32_t enc1_u, enc2_u;
  // We will cast to int32_t for printing since encoder counts are signed.

  // Attempt to read the encoder values for both motors
  // The function returns true on success, false on failure.
  // The values are stored in enc1_u and enc2_u.
  bool success = controller.ReadEncoders(MOTOR_ADDRESS, enc1_u, enc2_u);

  if (success) {
    Serial.println("GETENCODERS command successful.");
    Serial.print("Motor 1 Encoder: ");
    Serial.print((int32_t)enc1_u); // Cast to signed int32_t for correct representation
    Serial.print(", Motor 2 Encoder: ");
    Serial.println((int32_t)enc2_u); // Cast to signed int32_t for correct representation
  } else {
    Serial.println("GETENCODERS command failed.");
    Serial.println("Check wiring, power, address, baud rate, and controller configuration (Are encoders enabled?).");
  }

  // Wait a short time before reading again
  delay(100); // Reading encoder values frequently is common
}