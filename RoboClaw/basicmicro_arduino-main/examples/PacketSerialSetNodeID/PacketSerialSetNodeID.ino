/**
 * Basicmicro Library Example: SETNODEID (150) - MCP Only
 *
 * Demonstrates setting the CAN node ID for MCP series controllers that
 * support CAN communication. This ID is used to identify the controller
 * on a CAN network.
 *
 * The Node ID is an 8-bit value (0-255).
 *
 * !!! IMPORTANT: This command is marked as "MCP only". It may not work on
 * Sabertooth or Kangaroo controllers. Consult your controller's documentation
 * regarding CAN support and valid node ID ranges. !!!
 *
 * Note: This command sets the ID in the controller's RAM. To make it
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

// Define the desired CAN Node ID (0-255)
uint8_t desired_node_id = 10; // Example Node ID


void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro SETNODEID Example (MCP Only)");
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

  Serial.print("Attempting to set CAN Node ID to: ");
  Serial.println(desired_node_id);
  Serial.println("NOTE: This command is for MCP controllers with CAN support.");
}

void loop() {
  // Attempt to set the CAN Node ID
  // This command is typically only supported by MCP series controllers with CAN.
  // The function returns true on success, false on failure.
  bool success = controller.SetNodeID(MOTOR_ADDRESS, desired_node_id);

  if (success) {
    Serial.println("SETNODEID command successful. Node ID set in RAM.");
    Serial.println("Remember to use WRITENVM (94) to save permanently (causes reset).");
    // Optionally, read back the setting to verify (using GETNODEID)
    /*
    uint8_t read_node_id;
    if (controller.GetNodeID(MOTOR_ADDRESS, read_node_id)) { // Use the original address for this read
        Serial.print("Verified CAN Node ID: ");
        Serial.println(read_node_id); // Should match what we set
    } else {
        Serial.println("Failed to read CAN Node ID for verification.");
    }
    */

    // Example of saving to NVM afterwards (optional, and causes reset!)
    /*
    Serial.println("Attempting to save Node ID to NVM (WRITENVM)...");
    // Note: This will cause a reset and the controller will now respond on the NEW Node ID on CAN.
    // The serial address is unchanged by this command.
    if (controller.WriteNVM(MOTOR_ADDRESS)) { // Use the original serial address for WRITENVM
        Serial.println("WRITENVM command successful. Node ID saved.");
        Serial.println("Controller is now resetting. Wait a few seconds before sending more commands.");
        delay(5000); // Wait for controller to reset and boot
    } else {
        Serial.println("WRITENVM command failed. Node ID NOT saved.");
    }
    */

  } else {
    Serial.println("SETNODEID command failed.");
    Serial.println("Check wiring, power, address, baud rate, controller model, and configuration.");
    Serial.println("This command may not be supported by your controller or the Node ID is invalid.");
  }

  // Node ID is typically set once in setup or infrequently.
  // For this example, we'll add a long delay.
  delay(10000); // Wait 10 seconds before attempting again
}