/**
 * Basicmicro Library Example: GETENCSTATUS (103)
 *
 * Demonstrates reading the error status flags specifically for the encoders
 * on Motor 1 and Motor 2. This can indicate conditions like encoder overflow
 * or underflow.
 *
 * The returned values are 8-bit flags. Consult your controller's documentation
 * for the meaning of each bit in the status flags. Common status bits might
 * indicate overflow, underflow, or direction.
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

  Serial.println("Basicmicro GETENCSTATUS Example");
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

  Serial.println("Attempting to read encoder status flags...");
}

void loop() {
  // Variables to store the read encoder status flags (8-bit)
  uint8_t enc1_status, enc2_status;

  // Attempt to read the encoder status flags
  // The function returns true on success, false on failure.
  // The flags are stored in enc1_status and enc2_status.
  bool success = controller.GetEncStatus(MOTOR_ADDRESS, enc1_status, enc2_status);

  if (success) {
    Serial.println("GETENCSTATUS command successful.");
    Serial.println("Encoder Status Flags:");
    Serial.print("  Motor 1 Status: 0x"); Serial.println(enc1_status, HEX);
    Serial.print("  Motor 2 Status: 0x"); Serial.println(enc2_status, HEX);
    // Decode flags based on controller documentation. Example interpretation:
    if (enc1_status & 0x01) Serial.println("    M1: Underflow");
    if (enc1_status & 0x02) Serial.println("    M1: Overflow");
    if (enc1_status & 0x04) Serial.println("    M1: Direction Bit Set"); // This might correspond to status=3 in ReadEncM1
    // ... other potential bits depending on the controller

     if (enc2_status & 0x01) Serial.println("    M2: Underflow");
    if (enc2_status & 0x02) Serial.println("    M2: Overflow");
    if (enc2_status & 0x04) Serial.println("    M2: Direction Bit Set"); // This might correspond to status=3 in ReadEncM2
    // ... other potential bits depending on the controller

  } else {
    Serial.println("GETENCSTATUS command failed.");
    Serial.println("Check wiring, power, address, and baud rate.");
  }

  // Wait a short time before reading again
  delay(500); // Reading status periodically is common
}