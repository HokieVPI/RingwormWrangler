/**
 * Basicmicro Library Example: GETBUFFERS (47)
 *
 * Demonstrates reading the number of queued commands in the internal
 * command buffers for Motor 1 and Motor 2. This is useful when using
 * buffered commands (commands sent with the 'Add to buffer' flag set).
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

  Serial.println("Basicmicro GETBUFFERS Example");
  Serial.print("Connecting to controller on ");
  // Print the name of the serial port being used (if possible)
  #if defined(CONTROLLER_SERIAL) && !defined(RX_PIN) // Check if using HardwareSerial and not SoftwareSerial
    Serial.println("Hardware Serial");
  #elif defined(RX_PIN) // Check if SoftwareSerial pins are defined
    Serial.println("Software Serial");
  #else
    Serial.println("Unknown Serial type");
  #endif


  // Initialize the communication serial port for the controller
  controller.begin(9600); // Ensure baud rate matches your controller
  delay(100); // Short delay to let the controller initialize after power-up or serial init

  Serial.println("Attempting to read command buffer depths...");

  // Example: Send a command to add to buffer (using M1SPEEDDIST with bit 3 flag set)
  // This requires Velocity PID and encoder setup.
  uint32_t speed = 200;
  uint32_t distance = 1000;
  uint8_t  flag = 0; // 0 = buffer command, 1 = execute immediate

  Serial.println("Sending a buffered command for Motor 1 (assuming PID is set up)...");
  controller.SpeedDistanceM1(MOTOR_ADDRESS, speed, distance, flag);
  delay(100); // Give controller time to process command

  Serial.println("Sending another buffered command for Motor 1...");
  controller.SpeedDistanceM1(MOTOR_ADDRESS, speed, distance, flag);
  delay(100);

   Serial.println("Sending a buffered command for Motor 2...");
  controller.SpeedDistanceM2(MOTOR_ADDRESS, speed, distance, flag);
  delay(100);
}

void loop() {
  // Variables to store the read buffer depths
  uint8_t depth1, depth2;

  // Attempt to read the buffer depths
  // The function returns true on success, false on failure.
  // The depths are stored in depth1 and depth2.
  bool success = controller.ReadBuffers(MOTOR_ADDRESS, depth1, depth2);

  if (success) {
    Serial.println("GETBUFFERS command successful.");
    Serial.print("Motor 1 Buffer Depth: ");
    Serial.println(depth1);
    Serial.print("Motor 2 Buffer Depth: ");
    Serial.println(depth2);
  } else {
    Serial.println("GETBUFFERS command failed.");
    Serial.println("Check wiring, power, address, and baud rate.");
  }

  // Wait a short time before reading again
  delay(500);
}