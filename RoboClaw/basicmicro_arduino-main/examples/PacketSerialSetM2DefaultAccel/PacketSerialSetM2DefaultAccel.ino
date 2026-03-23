/**
 * Basicmicro Library Example: SETM2DEFAULTACCEL (69)
 *
 * Demonstrates setting the default acceleration and deceleration rates for Motor 2.
 * These rates are used by motion commands that do not explicitly specify acceleration
 * or deceleration parameters for Motor 2.
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

// Define example default acceleration and deceleration for Motor 2
// These values are in binary percents(32767 = 100% speed, 16384 = 50% etc...)
uint32_t default_m2_accel = 500; // Example default acceleration
uint32_t default_m2_decel = 700; // Example default deceleration


void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro SETM2DEFAULTACCEL Example");
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

  Serial.print("Attempting to set Motor 2 Default Accel: "); Serial.print(default_m2_accel);
  Serial.print(", Decel: "); Serial.println(default_m2_decel);
}

void loop() {
  // Attempt to set the default acceleration and deceleration for Motor 2
  // The function returns true on success, false on failure.
  bool success = controller.SetM2DefaultAccel(MOTOR_ADDRESS, default_m2_accel, default_m2_decel);

  if (success) {
    Serial.println("SETM2DEFAULTACCEL command successful.");
    // Optionally, read back the settings to verify (need GETDEFAULTACCELS)
    uint32_t read_accelM1, read_decelM1, read_accelM2, read_decelM2;
    if (controller.GetDefaultAccels(MOTOR_ADDRESS, read_accelM1, read_decelM1, read_accelM2, read_decelM2)) {
        Serial.println("Verified Default Accels:");
        Serial.print("  M1 Accel: "); Serial.print(read_accelM1);
        Serial.print(", M1 Decel: "); Serial.println(read_decelM1);
        Serial.print("  M2 Accel: "); Serial.print(read_accelM2);
        Serial.print(", M2 Decel: "); Serial.println(read_decelM2); // This should show the value we just set
    } else {
        Serial.println("Failed to read Default Accels for verification.");
    }
  } else {
    Serial.println("SETM2DEFAULTACCEL command failed.");
    Serial.println("Check wiring, power, address, and baud rate.");
  }

  // Default accel/decel are typically set once in setup or on configuration changes.
  // For this example, we'll add a long delay.
  delay(10000); // Wait 10 seconds before attempting again
}