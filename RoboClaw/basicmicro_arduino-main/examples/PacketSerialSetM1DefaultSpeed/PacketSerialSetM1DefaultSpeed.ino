/**
 * Basicmicro Library Example: SETM1DEFAULTSPEED (70)
 *
 * Demonstrates setting the default speed value for Motor 1.
 * This speed is used by commands that initiate movement for Motor 1
 * but do not explicitly specify a speed parameter (e.g., M1POS).
 * The speed value is in encoder counts per second (signed concept).
 *
 * This command requires Velocity PID control to be enabled and tuned for Motor 1.
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

// Define example default speed for Motor 1 in counts/sec (signed concept)
int32_t default_m1_speed = 16384; // Example default speed (50% speed)

void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro SETM1DEFAULTSPEED Example");
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

  // Note: For Speed commands to work, Velocity PID must be enabled and tuned.
  // You might need to send SETM1PID here or ensure it's configured in the controller's NVM.

  Serial.print("Attempting to set Motor 1 Default Speed to: "); Serial.print(default_m1_speed);
  Serial.println(" counts/sec");
}

void loop() {
  // Attempt to set the default speed for Motor 1
  // The function expects a uint32_t, so we cast our signed int32_t.
  bool success = controller.SetM1DefaultSpeed(MOTOR_ADDRESS, (uint16_t)default_m1_speed);

  if (success) {
    Serial.println("SETM1DEFAULTSPEED command successful.");
    // Optionally, read back the settings to verify (need GETDEFAULTSPEEDS)
    uint16_t read_speed1, read_speed2;
    if (controller.GetDefaultSpeeds(MOTOR_ADDRESS, read_speed1, read_speed2)) {
        Serial.println("Verified Default Speeds:");
        Serial.print("  M1 Default Speed: "); Serial.print((int16_t)read_speed1); // Cast back to signed for print
        Serial.print(", M2 Default Speed: "); Serial.println((int16_t)read_speed2); // Cast back to signed for print
    } else {
        Serial.println("Failed to read Default Speeds for verification.");
    }
  } else {
    Serial.println("SETM1DEFAULTSPEED command failed.");
    Serial.println("Check wiring, power, address, baud rate, and controller configuration.");
  }

  // Default speed is typically set once in setup or on configuration changes.
  // For this example, we'll add a long delay.
  delay(10000); // Wait 10 seconds before attempting again
}