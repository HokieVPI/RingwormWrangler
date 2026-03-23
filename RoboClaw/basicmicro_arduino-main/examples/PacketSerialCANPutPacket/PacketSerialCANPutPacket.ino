/**
 * Basicmicro Library Example: CANPUTPACKET (181) - MCP Only
 *
 * Demonstrates transmitting a CAN packet through the controller's CAN interface
 * on MCP series controllers with CAN support. This allows the Arduino to send
 * custom CAN messages via the Basicmicro controller acting as a gateway or node.
 *
 * You specify the CAN Object Identifier (cobID), Remote Transmission Request (RTR)
 * flag, data length (0-8 bytes), and the data bytes.
 *
 * !!! IMPORTANT: This command is marked as "MCP only" and requires CAN support.
 * It may not work on Sabertooth or Kangaroo controllers or MCPs without CAN.
 * Consult your controller's documentation regarding CAN configuration, valid
 * cobIDs, and data formats for the CAN network you are communicating on. !!!
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

// Define example CAN packet parameters
// !!! These are example values. You MUST use values appropriate for your CAN network. !!!
uint16_t example_cob_id = 0x181; // Example: A standard 11-bit CAN ID (e.g., a heartbeat message)
uint8_t  example_rtr_flag = 0;   // 0 for data frame, 1 for remote request frame
uint8_t  example_data_length = 4; // Length of the data payload (0-8 bytes)
uint8_t  example_data[8] = {0xAA, 0xBB, 0xCC, 0xDD, 0x00, 0x00, 0x00, 0x00}; // Example data bytes


void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro CANPUTPACKET Example (MCP Only)");
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

  Serial.println("Attempting to send a CAN packet...");
  Serial.println("NOTE: This command is for MCP controllers with CAN support.");
  Serial.print("CAN ID: 0x"); Serial.print(example_cob_id, HEX);
  Serial.print(", RTR: "); Serial.print(example_rtr_flag);
  Serial.print(", Length: "); Serial.print(example_data_length);
  Serial.print(", Data: ");
  for(uint8_t i=0; i<example_data_length; ++i) {
      if(example_data[i] < 0x10) Serial.print("0");
      Serial.print(example_data[i], HEX);
      Serial.print(" ");
  }
  Serial.println();
}

void loop() {
  // Attempt to send the CAN packet
  // This command is typically only supported by MCP series controllers with CAN.
  // The function returns true on success, false on failure (e.g., communication issue with Basicmicro controller).
  // It does NOT indicate if the CAN packet was successfully transmitted on the CAN bus or acknowledged by a CAN node.
  bool success = controller.CANPutPacket(MOTOR_ADDRESS,
                                          example_cob_id,
                                          example_rtr_flag,
                                          example_data_length,
                                          example_data);

  if (success) {
    Serial.println("CANPUTPACKET command successful (Sent to Basicmicro controller).");
  } else {
    Serial.println("CANPUTPACKET command failed (Communication with Basicmicro controller issue).");
    Serial.println("Check wiring, power, address, baud rate, controller model, and configuration.");
    Serial.println("This command may not be supported or CAN is not enabled.");
  }

  // Wait before attempting to send another packet
  delay(2000); // Send a packet every 2 seconds
}