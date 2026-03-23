/**
 * Basicmicro Library Example: SETPINFUNCTIONS (74)
 *
 * Demonstrates setting the operating mode for configurable I/O pins (S3, S4, S5, D1, D2).
 * The available modes depend on the specific controller model (e.g., digital input,
 * digital output, analog input, encoder input, limit switch, etc.).
 *
 * Consult your controller's documentation for the valid mode values for each pin.
 *
 * This example uses HardwareSerial (Serial1) for communication with the controller
 * and HardwareSerial (Serial) for debugging output. Adjust serial ports and
 * controller address as needed for your setup.`
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

// Define example modes for the configurable pins.
// !!! IMPORTANT: These values are examples. You MUST consult your controller's
// manual to find the correct mode values for your desired pin functions.
// Incorrect values may cause unexpected behavior. !!!
uint8_t s3_mode = 0; // Example: Mode 0 (e.g., Digital Input)
uint8_t s4_mode = 1; // Example: Mode 1 (e.g., Analog Input)
uint8_t s5_mode = 2; // Example: Mode 2 (e.g., Limit Switch)
uint8_t d1_mode = 0; // Example: Mode 0 (e.g., Digital Output)
uint8_t d2_mode = 0; // Example: Mode 0 (e.g., Digital Output)

void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro SETPINFUNCTIONS Example");
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

  Serial.println("Attempting to set pin functions...");
  Serial.print("S3: "); Serial.print(s3_mode);
  Serial.print(", S4: "); Serial.print(s4_mode);
  Serial.print(", S5: "); Serial.print(s5_mode);
  Serial.print(", D1: "); Serial.print(d1_mode);
  Serial.print(", D2: "); Serial.println(d2_mode);
}

void loop() {
  // Attempt to set the pin functions
  // The function returns true on success, false on failure.
  bool success = controller.SetPinFunctions(MOTOR_ADDRESS, s3_mode, s4_mode, s5_mode, d1_mode, d2_mode);

  if (success) {
    Serial.println("SETPINFUNCTIONS command successful.");
    // Optionally, read back the settings to verify (using GETPINFUNCTIONS)
    uint8_t read_s3, read_s4, read_s5, read_d1, read_d2;
    if (controller.GetPinFunctions(MOTOR_ADDRESS, read_s3, read_s4, read_s5, read_d1, read_d2)) {
        Serial.println("Verified Pin Functions:");
        Serial.print("  S3: "); Serial.print(read_s3);
        Serial.print(", S4: "); Serial.print(read_s4);
        Serial.print(", S5: "); Serial.print(read_s5);
        Serial.print(", D1: "); Serial.print(read_d1);
        Serial.print(", D2: "); Serial.println(read_d2);
    } else {
        Serial.println("Failed to read Pin Functions for verification.");
    }
  } else {
    Serial.println("SETPINFUNCTIONS command failed.");
    Serial.println("Check wiring, power, address, baud rate, and controller configuration.");
  }

  // Pin functions are typically set once in setup or on configuration changes.
  // For this example, we'll add a long delay.
  delay(10000); // Wait 10 seconds before attempting again
}