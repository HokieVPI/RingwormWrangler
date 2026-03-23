/**
 * Basicmicro Library Example: SETTIMEOUT (14)
 *
 * Demonstrates setting the communication timeout value on the Basicmicro motor controller.
 * The controller will enter a failsafe state if no commands are received within
 * the specified timeout period.
 *
 * This example uses HardwareSerial (Serial1) for communication with the controller
 * and HardwareSerial (Serial) for debugging output. Adjust serial ports and
 * controller address as needed for your setup.
 */

#include <Arduino.h>
#include <Basicmicro.h>

// Define the serial port for communication with the motor controller
// On boards like Mega, Due, Leonardo, etc., use Serial1, Serial2, Serial3.
// On boards like Uno, Nano, Micro, you might need SoftwareSerial if Serial is used for debug.
//#define CONTROLLER_SERIAL   Serial1

// Optional: Use SoftwareSerial on AVR boards if HardwareSerial is not available
 #include <SoftwareSerial.h>
 #define RX_PIN 10 // Connect to controller's TX pin
 #define TX_PIN 11 // Connect to controller's RX pin
 SoftwareSerial controllerSerial_SW(RX_PIN, TX_PIN);
 #define CONTROLLER_SERIAL   controllerSerial_SW // Use this define instead of Serial1

// Define the address of your motor controller
#define MOTOR_ADDRESS       128

// Define the communication timeout in microseconds (e.g., 100000 us = 100 ms)
// This is the library's internal read timeout, NOT the controller's communication timeout.
// The controller's timeout is set by the SETTIMEOUT command itself.
#define LIBRARY_READ_TIMEOUT 10000

// Instantiate the Basicmicro library object
Basicmicro controller(&CONTROLLER_SERIAL, LIBRARY_READ_TIMEOUT);

// The timeout value to set on the controller in seconds (0.0 to 25.5)
// Setting it to 1.5 seconds (15 tenths)
float controllerTimeoutSeconds = 1.5;

void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro SETTIMEOUT Example");
  Serial.print("Connecting to controller on ");
  // Print the name of the serial port being used (if possible)
  #if defined(CONTROLLER_SERIAL) && !defined(__AVR__)
    Serial.println("Hardware Serial");
  #elif defined(SSERIAL_RX_PIN) // Check if SoftwareSerial pins are defined
    Serial.println("Software Serial");
  #else
    Serial.println("Hardware Serial (check definition)");
  #endif

  // Initialize the communication serial port for the controller
  controller.begin(9600); // Ensure baud rate matches your controller
  delay(100); // Short delay to let the controller initialize after power-up or serial init

  Serial.println("Attempting to set controller timeout...");
  Serial.print("Setting timeout to ");
  Serial.print(controllerTimeoutSeconds);
  Serial.println(" seconds.");
}

void loop() {
  // Attempt to set the controller's communication timeout
  // The controller will enter failsafe if no commands are received within this time
  // Note: The command parameter is in tenths of a second (0-255)
  // The library's SetTimeout function handles this conversion.

  bool success = controller.SetTimeout(MOTOR_ADDRESS, controllerTimeoutSeconds);

  if (success) {
    Serial.println("SETTIMEOUT command successful.");
    // You might want to verify by reading it back
    float readTimeout;
    if (controller.GetTimeout(MOTOR_ADDRESS, readTimeout)) {
       Serial.print("Verified timeout is now: ");
       Serial.print(readTimeout);
       Serial.println(" seconds.");
    } else {
       Serial.println("Failed to read timeout for verification.");
    }
  } else {
    Serial.println("SETTIMEOUT command failed.");
    Serial.println("Check wiring, power, address, and baud rate.");
  }

  // This command doesn't need to be sent repeatedly unless you are changing the value.
  // We will just run it once in setup or have a long delay in loop.
  // For this example, we'll put a very long delay.
  delay(60000); // Wait 60 seconds before attempting again (or reset the board)
}
