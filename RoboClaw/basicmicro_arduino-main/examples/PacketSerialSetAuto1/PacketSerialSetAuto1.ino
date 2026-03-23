/**
 * Basicmicro Library Example: SETAUTO1 (105)
 *
 * Demonstrates setting the configuration value for "Auto Mode 1" on the
 * Basicmicro motor controller. The meaning and effect of this value are
 * highly dependent on the specific controller model and how "Auto Mode 1"
 * is implemented in its firmware.
 *
 * Consult your controller's documentation to understand the purpose and
 * valid range of this value.
 *
 * Note: This command sets the value in the controller's RAM. To make it
 * permanent so it persists across power cycles, you must send a
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

// Define the desired value for Auto Mode 1 (32-bit)
// !!! Replace this with a value derived from your controller's manual !!!
uint32_t auto1_value = 123456; // Example placeholder value

void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro SETAUTO1 Example");
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

  Serial.print("Attempting to set Auto Mode 1 value to: ");
  Serial.println(auto1_value);
  Serial.println("NOTE: The meaning of this value is specific to your controller model.");
}

void loop() {
  // Attempt to set the Auto Mode 1 value
  // The function returns true on success, false on failure.
  bool success = controller.SetAuto1(MOTOR_ADDRESS, auto1_value);

  if (success) {
    Serial.println("SETAUTO1 command successful. Value set in RAM.");
    Serial.println("Remember to use WRITENVM (94) to save permanently (causes reset).");
    // Optionally, read back the setting to verify (using GETAUTOS)
    /*
    uint32_t read_auto1, read_auto2;
    if (controller.GetAutos(MOTOR_ADDRESS, read_auto1, read_auto2)) {
        Serial.println("Verified Auto Mode Values:");
        Serial.print("  Auto Mode 1: "); Serial.println(read_auto1); // Should match what we set
        Serial.print("  Auto Mode 2: "); Serial.println(read_auto2); // Will show current Auto2 value
    } else {
        Serial.println("Failed to read Auto Mode Values for verification.");
    }
    */

    // Example of saving to NVM afterwards (optional, and causes reset!)
    /*
    Serial.println("Attempting to save value to NVM (WRITENVM)...");
    if (controller.WriteNVM(MOTOR_ADDRESS)) {
        Serial.println("WRITENVM command successful. Value saved.");
        Serial.println("Controller is now resetting. Wait a few seconds before sending more commands.");
        delay(5000); // Wait for controller to reset and boot
    } else {
        Serial.println("WRITENVM command failed. Value NOT saved.");
    }
    */

  } else {
    Serial.println("SETAUTO1 command failed.");
    Serial.println("Check wiring, power, address, baud rate, and controller configuration.");
  }

  // This command is typically set once in setup or infrequently.
  // For this example, we'll add a long delay.
  delay(10000); // Wait 10 seconds before attempting again
}