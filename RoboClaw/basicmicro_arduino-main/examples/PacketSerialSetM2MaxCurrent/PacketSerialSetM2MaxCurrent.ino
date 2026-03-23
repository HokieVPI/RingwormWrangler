/**
 * Basicmicro Library Example: SETM2MAXCURRENT (134)
 *
 * Demonstrates setting the maximum allowable current limits for Motor 2.
 * These limits (for driving and regenerative braking) protect the motor
 * and controller from excessive current draw. If the current exceeds a limit,
 * the controller will restrict power to the motor.
 *
 * The current values are specified in 10s of milliamps (mA).
 *
 * Note: This command sets the limits in the controller's RAM. To make them
 * permanent so they persist across power cycles, you must send a
 * WRITENVM (94) command afterwards. Be aware that WRITENVM will cause the
 * controller to reset.
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

// Define example maximum current limits for Motor 2 in 10s of milliamps (uint32_t)
// !!! These values are examples. You MUST use values appropriate for your motor. !!!
uint32_t m2_max_current_mA = 2200; // Example: 22 Amps Driving Current Limit
uint32_t m2_min_current_mA = 1100; // Example: 11 Amps Regenerative Braking Current Limit


void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro SETM2MAXCURRENT Example");
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

  Serial.print("Attempting to set Motor 2 Max Current (Driving/Regen) to: ");
  Serial.print(m2_max_current_mA); Serial.print(" mA / ");
  Serial.print(m2_min_current_mA); Serial.println(" mA");
}

void loop() {
  // Attempt to set the maximum current limits for Motor 2
  // The function expects uint32_t values for both max and min limits.
  bool success = controller.SetM2MaxCurrent(MOTOR_ADDRESS, m2_max_current_mA, m2_min_current_mA);

  if (success) {
    Serial.println("SETM2MAXCURRENT command successful. Limits set in RAM.");
    Serial.println("Remember to use WRITENVM (94) to save permanently (causes reset).");
    // Optionally, read back the setting to verify (using ReadM2MaxCurrent)
    /*
    uint32_t read_max, read_min;
    if (controller.ReadM2MaxCurrent(MOTOR_ADDRESS, read_max, read_min)) {
        Serial.println("Verified Motor 2 Max Current (Driving/Regen):");
        Serial.print("  Max: "); Serial.print(read_max); Serial.print(" mA, ");
        Serial.print("Min: "); Serial.print(read_min); Serial.println(" mA");
    } else {
        Serial.println("Failed to read Motor 2 Max Current for verification.");
    }
    */

    // Example of saving to NVM afterwards (optional, and causes reset!)
    /*
    Serial.println("Attempting to save limits to NVM (WRITENVM)...");
    if (controller.WriteNVM(MOTOR_ADDRESS)) {
        Serial.println("WRITENVM command successful. Limits saved.");
        Serial.println("Controller is now resetting. Wait a few seconds before sending more commands.");
        delay(5000); // Wait for controller to reset and boot
    } else {
        Serial.println("WRITENVM command failed. Limits NOT saved.");
    }
    */

  } else {
    Serial.println("SETM2MAXCURRENT command failed.");
    Serial.println("Check wiring, power, address, baud rate, and controller configuration.");
  }

  // Max current limits are typically set once in setup or on configuration changes.
  // For this example, we'll add a long delay.
  delay(10000); // Wait 10 seconds before attempting again
}