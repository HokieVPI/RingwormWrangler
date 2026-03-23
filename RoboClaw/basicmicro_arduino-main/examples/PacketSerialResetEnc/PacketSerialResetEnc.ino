/**
 * Basicmicro Library Example: RESETENC (20)
 *
 * Demonstrates resetting the encoder count values for both Motor 1 and Motor 2
 * on the Basicmicro motor controller. This sets the current position of both
 * motors to zero, which is useful for establishing a reference point for
 * position control.
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

  Serial.println("Basicmicro RESETENC Example");
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

  Serial.println("Attempting to reset encoders...");
}

void loop() {
  // Attempt to reset both encoder counters to zero
  // The function returns true on success, false on failure.
  bool success = controller.ResetEncoders(MOTOR_ADDRESS);

  if (success) {
    Serial.println("RESETENC command successful. Encoders should now be zero.");
    // You might want to read encoder values to verify
    uint32_t enc1, enc2;
    if (controller.ReadEncoders(MOTOR_ADDRESS, enc1, enc2)) {
        Serial.print("Verified M1 Enc: ");
        Serial.print((int32_t)enc1);
        Serial.print(", M2 Enc: ");
        Serial.println((int32_t)enc2);
    } else {
        Serial.println("Failed to read encoders after reset.");
    }
  } else {
    Serial.println("RESETENC command failed.");
    Serial.println("Check wiring, power, address, and baud rate.");
  }

  // This command typically doesn't need to be sent repeatedly.
  // Delay for a long time or remove the loop functionality for a single reset.
  delay(10000); // Wait 10 seconds before attempting again
}