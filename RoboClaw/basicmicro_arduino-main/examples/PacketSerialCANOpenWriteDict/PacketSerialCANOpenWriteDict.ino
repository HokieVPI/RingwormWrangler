/**
 * Basicmicro Library Example: CANOPENWRITEDICT (190) - MCP Only
 *
 * Demonstrates writing a value to an entry in the CANopen Object Dictionary
 * on MCP series controllers with CANopen support. This allows configuring
 * CANopen nodes (including the Basicmicro controller itself if addressed with NodeID 0).
 *
 * You specify the target Node ID (0 for local), Dictionary Index, Subindex,
 * the value to write (up to 32-bit), and the size of the value (1, 2, or 4 bytes).
 *
 * !!! IMPORTANT: This command is marked as "MCP only" and requires CANopen support.
 * It may not work on Sabertooth or Kangaroo controllers or MCPs without CANopen.
 * Consult your controller's documentation and the target device's EDS file
 * for valid Indexes, Subindexes, sizes, and value ranges. Incorrect parameters
 * can cause unexpected behavior or communication errors. !!!
 *
 * Note: Writing to the local dictionary (NodeID 0) modifies the controller's RAM settings.
 * To make these changes permanent so they persist across power cycles, you must
 * send a WRITENVM (94) command afterwards. Be aware that WRITENVM will cause the
 * controller to reset. Writing to a remote Node ID modifies the target device's
 * configuration (behavior depends on that device's firmware).
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

// Define the address of your Basicmicro controller (acting as gateway/master)
#define BASICMICRO_ADDRESS       128

// Define the library's internal read timeout in microseconds
#define LIBRARY_READ_TIMEOUT 10000

// Instantiate the Basicmicro library object
// If using SoftwareSerial, uncomment the #define above and use controllerSerial_SW
Basicmicro controller(&CONTROLLER_SERIAL, LIBRARY_READ_TIMEOUT);

// Define example CANopen dictionary entry to write to
// !!! These are example values. You MUST consult the target device's documentation. !!!
uint8_t  target_node_id = 0;       // Example: 0 for the Basicmicro controller's local dictionary
                                   // Change to a remote node ID (e.g., 1-127) to configure another device
uint16_t dict_index = 0x2000;      // Example Dictionary Index (e.g., a vendor-specific object)
uint8_t  dict_subindex = 1;      // Example Subindex
uint32_t value_to_write = 12345; // Example value to write (sent as 32-bit)
uint8_t  value_size = 4;         // Example size (1, 2, or 4 bytes). Must match object definition.


void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro CANOPENWRITEDICT Example (MCP Only)");
  Serial.print("Connecting to Basicmicro controller on ");
  // Print the name of the serial port being used (if possible)
  #if defined(CONTROLLER_SERIAL) && !defined(RX_PIN) // Check if using HardwareSerial and not SoftwareSerial
    Serial.println("Hardware Serial");
  #elif defined(RX_PIN) // Check if SoftwareSerial pins are defined using RX_PIN
    Serial.println("Software Serial");
  #else
    Serial.println("Unknown Serial type");
  #endif


  // Initialize the communication serial port for the Basicmicro controller
  controller.begin(9600); // Ensure baud rate matches your Basicmicro controller
  delay(100); // Short delay

  Serial.println("Attempting to write to CANopen dictionary...");
  Serial.println("NOTE: This command is for MCP controllers with CANopen support.");
  Serial.print("Target Node ID: "); Serial.print(target_node_id);
  Serial.print(", Dictionary Index: 0x"); Serial.print(dict_index, HEX);
  Serial.print(", Subindex: "); Serial.print(dict_subindex);
  Serial.print(", Value: "); Serial.print(value_to_write);
  Serial.print(", Size: "); Serial.println(value_size);
  Serial.println("Consult device documentation for valid parameters.");
}

void loop() {
  // Variable to store the operation result code (32-bit)
  uint32_t operation_result;

  // Attempt to write to the CANopen dictionary
  // This command is typically only supported by MCP series controllers with CANopen.
  // The function returns true on successful communication with the Basicmicro controller,
  // and stores the CANopen operation result code in 'operation_result'.
  // The 'operation_result' indicates the success/failure of the CANopen transaction itself.
  bool success = controller.CANOpenWriteDict(BASICMICRO_ADDRESS,
                                             target_node_id,
                                             dict_index,
                                             dict_subindex,
                                             value_to_write,
                                             value_size,
                                             operation_result);

  if (success) {
    Serial.println("CANOPENWRITEDICT command successful (Communication with Basicmicro controller ok).");
    Serial.print("CANopen Operation Result Code: 0x");
    Serial.println(operation_result, HEX); // Consult CANopen documentation for interpretation
    // A result code of 0 typically indicates successful SDO transfer.
    if (operation_result == 0) {
        Serial.println("  -> CANopen SDO write successful.");
        if (target_node_id == 0) {
             Serial.println("  Settings updated in Basicmicro controller's RAM.");
             Serial.println("  Remember to use WRITENVM (94) to save permanently (causes reset).");
        } else {
             Serial.print("  Settings updated on remote device Node ID "); Serial.println(target_node_id);
             // If configuring a remote device, you might need a command to save its NVM as well (if supported).
        }
    } else {
        Serial.println("  -> CANopen SDO write failed (check result code).");
    }

    // Example of saving local changes to NVM afterwards (optional, and causes reset!)
    // Only if target_node_id is 0 (writing to local dictionary)
    /*
    if (target_node_id == 0) {
        Serial.println("Attempting to save local changes to NVM (WRITENVM)...");
        if (controller.WriteNVM(BASICMICRO_ADDRESS)) {
            Serial.println("WRITENVM command successful. Local config saved.");
            Serial.println("Basicmicro Controller is now resetting. Wait a few seconds before sending more commands.");
            delay(5000); // Wait for controller to reset and boot
        } else {
            Serial.println("WRITENVM command failed. Local config NOT saved.");
        }
    }
    */

  } else {
    Serial.println("CANOPENWRITEDICT command failed (Communication with Basicmicro controller issue).");
    Serial.println("Check wiring, power, address, baud rate, controller model, and configuration.");
    Serial.println("This command may not be supported or CANopen is not enabled/configured correctly.");
  }

  // Writing dictionary values is typically a configuration step, done once.
  // For this example, the loop will pause indefinitely after the first attempt.
  while(true); // Stop execution after setup and the single command attempt
}