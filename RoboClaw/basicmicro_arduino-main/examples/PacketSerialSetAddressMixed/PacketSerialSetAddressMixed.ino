/**
 * Basicmicro Library Example: SETADDRESSMIXED (141) - MCP Only
 *
 * Demonstrates setting the controller's address and enabling/disabling
 * addressed mixing mode on MCP series controllers. Addressed mixing allows
 * control of a single motor from two different sources (addresses) for
 * mixed-mode control (like tank steering).
 *
 * !!! IMPORTANT: This command is marked as "MCP only". It may not work on
 * Sabertooth or Kangaroo controllers. Consult your controller's documentation
 * regarding addressed mixing and its setup. !!!
 *
 * Note: This command sets the values in the controller's RAM. To make them
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

// Define the CURRENT address of your motor controller
#define MOTOR_ADDRESS       128

// Define the library's internal read timeout in microseconds
#define LIBRARY_READ_TIMEOUT 10000

// Instantiate the Basicmicro library object
// If using SoftwareSerial, uncomment the #define above and use controllerSerial_SW
Basicmicro controller(&CONTROLLER_SERIAL, LIBRARY_READ_TIMEOUT);

// Define the desired new address and mixing enable state
uint8_t new_controller_address = 129; // Example: Change address to 129
uint8_t enable_mixing_state = 1;    // Example: 1 to enable addressed mixing, 0 to disable


void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro SETADDRESSMIXED Example (MCP Only)");
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

  Serial.print("Attempting to set new address to: "); Serial.print(new_controller_address);
  Serial.print(" and enable mixing: "); Serial.println(enable_mixing_state);
  Serial.println("NOTE: This command is for MCP controllers.");
}

void loop() {
  // Attempt to set the new address and mixing state
  // This command is typically only supported by MCP series controllers.
  // Note: If you change the address successfully, future commands in this sketch
  // will need to use the NEW address (if you put this logic in loop) or you
  // will lose communication! This example runs only once in loop.
  // The function returns true on success, false on failure.
  bool success = controller.SetAddressMixed(MOTOR_ADDRESS, new_controller_address, enable_mixing_state);

  if (success) {
    Serial.println("SETADDRESSMIXED command successful. Settings set in RAM.");
    Serial.print("Controller address is now set to "); Serial.print(new_controller_address);
    Serial.print(" with mixing "); Serial.println(enable_mixing_state ? "ENABLED" : "DISABLED");
    Serial.println("Remember to use WRITENVM (94) to save permanently (causes reset).");

    // After changing the address, the original address (MOTOR_ADDRESS) will no longer work.
    // Future commands must use new_controller_address.

    // Example of saving to NVM afterwards (optional, and causes reset!)
    /*
    Serial.println("Attempting to save settings to NVM (WRITENVM)...");
    // Note: If using this, ensure you handle the reset and potentially re-establish comms.
    if (controller.WriteNVM(new_controller_address)) { // Use the NEW address
        Serial.println("WRITENVM command successful. Settings saved.");
        Serial.println("Controller is now resetting. Wait a few seconds before sending more commands.");
        delay(5000); // Wait for controller to reset and boot
    } else {
        Serial.println("WRITENVM command failed. Settings NOT saved.");
    }
    */

  } else {
    Serial.println("SETADDRESSMIXED command failed.");
    Serial.println("Check wiring, power, address, baud rate, controller model, and configuration.");
    Serial.println("This command may not be supported by your controller or parameters are invalid.");
  }

  // This command is typically set once in setup or very infrequently.
  // For this example, the loop will pause indefinitely after the first attempt.
  while(true); // Stop execution after setup and the single command attempt
}