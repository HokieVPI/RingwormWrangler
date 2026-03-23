/**
 * Basicmicro Library Example: SETSCRIPTAUTORUN (246) - MCP Only
 *
 * Demonstrates setting the script autorun time on MCP series controllers
 * that support onboard scripting. This configures the delay in milliseconds
 * after startup before a configured script begins execution.
 *
 * A value of 0 typically disables script autorun. Values less than 100
 * are often treated as 0.
 *
 * !!! IMPORTANT: This command is marked as "MCP only" and requires scripting support.
 * It may not work on Sabertooth or Kangaroo controllers or MCPs without scripting.
 * Consult your controller's documentation. !!!
 *
 * Note: This command sets the time in the controller's RAM. To make it
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

// Define the desired script autorun time in milliseconds
uint32_t autorun_time_ms = 3000; // Example: Autorun script after 3 seconds (3000 ms)
// Set to 0 to disable autorun.


void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro SETSCRIPTAUTORUN Example (MCP Only)");
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
  delay(100); // Short delay

  Serial.print("Attempting to set Script Autorun Time to: "); Serial.print(autorun_time_ms); Serial.println(" ms");
  Serial.println("NOTE: This command is for MCP controllers with scripting support.");
}

void loop() {
  // Attempt to set the script autorun time
  // This command is typically only supported by MCP series controllers with scripting.
  // The function returns true on success, false on failure.
  bool success = controller.SetScriptAutorun(MOTOR_ADDRESS, autorun_time_ms);

  if (success) {
    Serial.println("SETSCRIPTAUTORUN command successful. Autorun time set in RAM.");
    Serial.println("Remember to use WRITENVM (94) to save permanently (causes reset).");
    // Optionally, read back the setting to verify (using GETSCRIPTAUTORUN)
    /*
    uint32_t read_autorun_time;
    if (controller.GetScriptAutorun(MOTOR_ADDRESS, read_autorun_time)) {
        Serial.print("Verified Script Autorun Time: ");
        Serial.print(read_autorun_time); Serial.println(" ms"); // Should match what we set
    } else {
        Serial.println("Failed to read Script Autorun Time for verification.");
    }
    */

    // Example of saving to NVM afterwards (optional, and causes reset!)
    /*
    Serial.println("Attempting to save autorun time to NVM (WRITENVM)...");
    if (controller.WriteNVM(MOTOR_ADDRESS)) {
        Serial.println("WRITENVM command successful. Autorun time saved.");
        Serial.println("Controller is now resetting. Wait a few seconds before sending more commands.");
        delay(5000); // Wait for controller to reset and boot
    } else {
        Serial.println("WRITENVM command failed. Autorun time NOT saved.");
    }
    */

  } else {
    Serial.println("SETSCRIPTAUTORUN command failed.");
    Serial.println("Check wiring, power, address, baud rate, controller model, and configuration.");
    Serial.println("This command may not be supported by your controller or the value is invalid.");
  }

  // Script autorun time is typically set once in setup or infrequently.
  // For this example, we'll add a long delay.
  delay(10000); // Wait 10 seconds before attempting again
}