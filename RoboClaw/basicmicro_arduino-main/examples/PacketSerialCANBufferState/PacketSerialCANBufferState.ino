/**
 * Basicmicro Library Example: CANBUFFERSTATE (180) - MCP Only
 *
 * Demonstrates reading the number of pending CAN packets in the controller's
 * internal transmit or receive buffer (depending on controller configuration)
 * on MCP series controllers with CAN support.
 *
 * This can be useful for monitoring the CAN bus activity or ensuring packets
 * are being processed.
 *
 * !!! IMPORTANT: This command is marked as "MCP only" and requires CAN support.
 * It may not work on Sabertooth or Kangaroo controllers or MCPs without CAN.
 * Consult your controller's documentation. !!!
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


void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro CANBUFFERSTATE Example (MCP Only)");
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

  Serial.println("Attempting to read CAN buffer state...");
  Serial.println("NOTE: This command is for MCP controllers with CAN support.");

  // To see the buffer state change, you might need to send/receive CAN packets
  // using CANPUTPACKET (181) or CANGETPACKET (182).
}

void loop() {
  // Variable to store the number of pending CAN packets (unsigned 8-bit)
  uint8_t packet_count = 0;

  // Attempt to read the CAN buffer state
  // This command is typically only supported by MCP series controllers with CAN.
  // The function returns true on success, false on failure.
  // The number of packets is stored in packet_count.
  bool success = controller.CANBufferState(MOTOR_ADDRESS, packet_count);

  if (success) {
    Serial.println("CANBUFFERSTATE command successful.");
    Serial.print("Pending CAN Packets in Buffer: ");
    Serial.println(packet_count); // Number of packets (units depend on controller implementation)
  } else {
    Serial.println("CANBUFFERSTATE command failed.");
    Serial.println("Check wiring, power, address, baud rate, controller model, and configuration.");
    Serial.println("This command may not be supported by your controller or CAN is not enabled.");
  }

  // Wait a short time before reading again
  delay(500); // Reading buffer state periodically is common for monitoring
}