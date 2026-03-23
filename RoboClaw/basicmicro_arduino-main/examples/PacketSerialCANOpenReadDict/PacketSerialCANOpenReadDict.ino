/**
 * Basicmicro Library Example: CANOPENREADDICT (191) - MCP Only
 *
 * Demonstrates reading a value from an entry in the CANopen Object Dictionary
 * on MCP series controllers with CANopen support. This allows reading configuration
 * or status values from CANopen nodes (including the Basicmicro controller itself
 * if addressed with NodeID 0).
 *
 * You specify the target Node ID (0 for local), Dictionary Index, and Subindex.
 * You receive the read value (as 32-bit, size parameter indicates actual size),
 * the actual size and type of the object, and the CANopen operation result code.
 *
 * !!! IMPORTANT: This command is marked as "MCP only" and requires CANopen support.
 * It may not work on Sabertooth or Kangaroo controllers or MCPs without CANopen.
 * Consult your controller's documentation and the target device's EDS file
 * for valid Indexes, Subindexes, sizes, and types. Incorrect parameters
 * can cause unexpected behavior or communication errors. !!!
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

// Define example CANopen dictionary entry to read from
// !!! These are example values. You MUST consult the target device's documentation. !!!
uint8_t  target_node_id = 0;       // Example: 0 for the Basicmicro controller's local dictionary
                                   // Change to a remote node ID (e.g., 1-127) to read from another device
uint16_t dict_index = 0x2000;      // Example Dictionary Index (e.g., a vendor-specific object)
uint8_t  dict_subindex = 1;      // Example Subindex


void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro CANOPENREADDICT Example (MCP Only)");
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

  Serial.println("Attempting to read from CANopen dictionary...");
  Serial.println("NOTE: This command is for MCP controllers with CANopen support.");
  Serial.print("Target Node ID: "); Serial.print(target_node_id);
  Serial.print(", Dictionary Index: 0x"); Serial.print(dict_index, HEX);
  Serial.print(", Subindex: "); Serial.println(dict_subindex);
  Serial.println("Consult device documentation for valid parameters.");
}

void loop() {
  // Variables to store the read value and operation result
  uint32_t read_value;    // The value read (stored in 32-bit format)
  uint8_t  actual_size;   // The actual size of the object (1, 2, or 4 bytes)
  uint8_t  data_type;     // The data type of the object (vendor-specific?)
  uint32_t operation_result; // The CANopen operation result code (32-bit)

  // Attempt to read from the CANopen dictionary
  // This command is typically only supported by MCP series controllers with CANopen.
  // The function returns true on successful communication with the Basicmicro controller,
  // and fills the result variables based on the CANopen transaction response.
  bool success = controller.CANOpenReadDict(BASICMICRO_ADDRESS,
                                            target_node_id,
                                            dict_index,
                                            dict_subindex,
                                            read_value,
                                            actual_size,
                                            data_type,
                                            operation_result);

  if (success) {
    Serial.println("CANOPENREADDICT command successful (Communication with Basicmicro controller ok).");
    Serial.print("CANopen Operation Result Code: 0x");
    Serial.println(operation_result, HEX); // Consult CANopen documentation for interpretation

    // A result code of 0 typically indicates successful SDO transfer.
    if (operation_result == 0) {
        Serial.println("  -> CANopen SDO read successful.");
        Serial.print("  Read Value (as uint32_t): "); Serial.println(read_value);
        Serial.print("  Actual Size (bytes): "); Serial.println(actual_size);
        Serial.print("  Data Type: "); Serial.println(data_type); // Consult device documentation for meaning
        Serial.println("  NOTE: You may need to cast the read value to its actual type based on 'actual_size'.");
        switch(actual_size) {
            case 1: Serial.print("    (Value as uint8_t: "); Serial.print((uint8_t)read_value); Serial.println(")"); break;
            case 2: Serial.print("    (Value as uint16_t: "); Serial.print((uint16_t)read_value); Serial.println(")"); break;
            case 4: /* already printed as uint32_t */ break;
            // Handle signed types if necessary by casting
            // case 1: Serial.print("    (Value as int8_t: "); Serial.print((int8_t)read_value); Serial.println(")"); break;
            // case 2: Serial.print("    (Value as int16_t: "); Serial.print((int16_t)read_value); Serial.println(")"); break;
            // case 4: Serial.print("    (Value as int32_t: "); Serial.print((int32_t)read_value); Serial.println(")"); break;
        }
    } else {
        Serial.println("  -> CANopen SDO read failed (check result code).");
    }

  } else {
    Serial.println("CANOPENREADDICT command failed (Communication with Basicmicro controller issue).");
    Serial.println("Check wiring, power, address, baud rate, controller model, and configuration.");
    Serial.println("This command may not be supported or CANopen is not enabled/configured correctly.");
  }

  // Wait a few seconds before reading again
  delay(2000); // Reading dictionary values periodically might be needed depending on the object
}