/**
 * Basicmicro Library Example: SETM2ENCODERMODE (93)
 *
 * Demonstrates setting the encoder interface mode for Motor 2.
 * This determines how the controller interprets signals from the encoder connected to M2.
 *
 * Mode values:
 * 0 = Quadrature encoder (standard, uses both channels A & B)
 * 1 = Single-ended (pulse/direction mode, uses one channel for pulses, another for direction)
 *
 * Consult your controller's documentation for supported modes and connection details.
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

// Define the desired encoder mode for Motor 2
// Choose 0 for Quadrature or 1 for Single-ended
uint8_t motor2_encoder_mode = 1; // Example: Set to Quadrature mode


void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro SETM2ENCODERMODE Example");
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

  Serial.print("Attempting to set Motor 2 Encoder Mode to: ");
  Serial.println(motor2_encoder_mode); // 0=Quadrature, 1=Single-ended
}

void loop() {
  // Attempt to set the encoder mode for Motor 2
  // The function returns true on success, false on failure.
  bool success = controller.SetM2EncoderMode(MOTOR_ADDRESS, motor2_encoder_mode);

  if (success) {
    Serial.println("SETM2ENCODERMODE command successful.");
    // Optionally, read back the setting to verify (using GETENCODERMODE)
    uint8_t read_m1_mode, read_m2_mode;
    if (controller.ReadEncoderModes(MOTOR_ADDRESS, read_m1_mode, read_m2_mode)) {
        Serial.println("Verified Encoder Modes:");
        Serial.print("  Motor 1 Mode: "); Serial.println(read_m1_mode); // Will show current M1 mode
        Serial.print("  Motor 2 Mode: "); Serial.println(read_m2_mode); // Should match what we set
    } else {
        Serial.println("Failed to read Encoder Modes for verification.");
    }
  } else {
    Serial.println("SETM2ENCODERMODE command failed.");
    Serial.println("Check wiring, power, address, baud rate, and controller configuration.");
  }

  // Encoder mode is typically set once in setup or on configuration changes.
  // For this example, we'll add a long delay.
  delay(10000); // Wait 10 seconds before attempting again
}