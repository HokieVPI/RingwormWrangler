/**
 * Basicmicro Library Example: CANGETPACKET (182) - MCP Only
 *
 * Demonstrates receiving a CAN packet from the controller's internal receive
 * buffer on MCP series controllers with CAN support. This allows the Arduino
 * to receive CAN messages filtered and buffered by the Basicmicro controller.
 *
 * You read the CAN Object Identifier (cobID), Remote Transmission Request (RTR)
 * flag, data length (0-8 bytes), and the data bytes.
 *
 * !!! IMPORTANT: This command is marked as "MCP only" and requires CAN support.
 * It may not work on Sabertooth or Kangaroo controllers or MCPs without CAN.
 * Consult your controller's documentation regarding CAN configuration, receive
 * filters, and buffering. You must configure the MCP to receive the desired CAN IDs. !!!
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

// Buffer to store received CAN data (max 8 bytes)
uint8_t received_data[8];


void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro CANGETPACKET Example (MCP Only)");
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

  Serial.println("Attempting to read CAN packets from buffer...");
  Serial.println("NOTE: This command is for MCP controllers with CAN support.");
  Serial.println("Waiting for incoming CAN packets...");

  // Ensure CAN receive is configured on the controller, and packets are being sent on the CAN bus.
}

void loop() {
  // Variables to store the received packet information
  uint16_t cob_id;
  uint8_t  rtr_flag;
  uint8_t  data_length;

  // Attempt to get a CAN packet from the buffer
  // The function returns true if a packet was successfully received from the Basicmicro controller,
  // false if there was a communication error OR if no packet was available in the buffer.
  // You may want to check CANBUFFERSTATE (180) first to see if packets are pending.
  bool success = controller.CANGetPacket(MOTOR_ADDRESS,
                                         cob_id,
                                         rtr_flag,
                                         data_length,
                                         received_data);

  if (success) {
    Serial.println("CANGETPACKET command successful. Packet received.");
    Serial.print("  CAN ID: 0x"); Serial.print(cob_id, HEX);
    Serial.print(", RTR: "); Serial.print(rtr_flag);
    Serial.print(", Length: "); Serial.print(data_length);
    Serial.print(", Data: ");
    // Print the received data bytes up to the reported length (max 8)
    for(uint8_t i=0; i<data_length; ++i) {
        if(received_data[i] < 0x10) Serial.print("0");
        Serial.print(received_data[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

  } else {
    // If it failed, it could be a communication error OR no packet in buffer.
    // The library doesn't provide a direct way to distinguish here without reading status bits.
    // Assume it's likely just an empty buffer if communication is generally stable.
    // Serial.println("CANGETPACKET command failed or no packet available.");
    // Omit this line to avoid flooding serial if buffer is empty and loop is fast.
  }

  // Wait a short time before trying to read the next packet
  delay(10); // Check for new packets frequently
}