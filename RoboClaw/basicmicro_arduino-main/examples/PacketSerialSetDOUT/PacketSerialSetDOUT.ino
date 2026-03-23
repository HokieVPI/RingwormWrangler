/**
 * Basicmicro Library Example: SETDOUT (137) - MCP Only
 *
 * Demonstrates setting the action for a specific digital output pin by its index
 * on MCP series controllers. The available indexes and actions depend on the
 * specific MCP model and its configuration.
 *
 * !!! IMPORTANT: This command is marked as "MCP only". It may not work on
 * Sabertooth or Kangaroo controllers. Consult your controller's documentation
 * regarding available output indexes and their corresponding action values. !!!
 *
 * Note: This command sets the action in the controller's RAM. To make it
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

// Define example parameters for setting a digital output
// !!! Consult your controller's manual for valid indexes and actions !!!
uint8_t output_index = 0; // Example: Digital Output 0 (Check documentation)
uint8_t output_action = 1; // Example: Action 1 (e.g., Turn ON, Check documentation)


void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro SETDOUT Example (MCP Only)");
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

  Serial.print("Attempting to set Digital Output Index "); Serial.print(output_index);
  Serial.print(" Action to: "); Serial.println(output_action);
  Serial.println("NOTE: This command is for MCP controllers and parameters are model-specific.");
}

void loop() {
  // Attempt to set the digital output action
  // This command is typically only supported by MCP series controllers.
  // The function returns true on success, false on failure.
  bool success = controller.SetDOUT(MOTOR_ADDRESS, output_index, output_action);

  if (success) {
    Serial.println("SETDOUT command successful. Action set in RAM.");
    Serial.println("Remember to use WRITENVM (94) to save permanently (causes reset).");
    // Optionally, read back the setting to verify (using GETDOUTS - note it reads ALL outputs)
    /*
    uint8_t count;
    uint8_t actions[5]; // Assuming max 5 digital outputs, adjust as needed
    if (controller.GetDOUTS(MOTOR_ADDRESS, count, actions, sizeof(actions))) {
        Serial.println("Verified Digital Output Actions:");
        for (uint8_t i = 0; i < count; i++) {
            Serial.print("  Output "); Serial.print(i); Serial.print(": "); Serial.println(actions[i]);
        }
    } else {
        Serial.println("Failed to read Digital Output Actions for verification.");
    }
    */

    // Example of saving to NVM afterwards (optional, and causes reset!)
    /*
    Serial.println("Attempting to save DOUT settings to NVM (WRITENVM)...");
    if (controller.WriteNVM(MOTOR_ADDRESS)) {
        Serial.println("WRITENVM command successful. Settings saved.");
        Serial.println("Controller is now resetting. Wait a few seconds before sending more commands.");
        delay(5000); // Wait for controller to reset and boot
    } else {
        Serial.println("WRITENVM command failed. Settings NOT saved.");
    }
    */

  } else {
    Serial.println("SETDOUT command failed.");
    Serial.println("Check wiring, power, address, baud rate, controller model, and configuration.");
    Serial.println("This command may not be supported by your controller or parameters are invalid.");
  }

  // Digital output settings are typically set once in setup or infrequently.
  // For this example, we'll add a long delay.
  delay(10000); // Wait 10 seconds before attempting again
}