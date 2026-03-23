/**
 * Basicmicro Library Example: GETSPEEDS (108)
 *
 * Demonstrates reading the current speed values for both Motor 1 and Motor 2
 * in a single command. This is a convenience command that provides the filtered
 * speed measurements, similar to calling GETM1SPEED and GETM2SPEED, but in one request.
 *
 * The speeds are in encoder counts per second (signed concept, but returned as uint32_t).
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

  Serial.println("Basicmicro GETSPEEDS Example");
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

  Serial.println("Attempting to read speeds for both motors...");

  // Example: Command motors to move to see speeds
  // Assuming Velocity PID and encoders are set up, command a speed:
  // controller.SpeedM1M2(MOTOR_ADDRESS, 500, -500); // Move M1 forward, M2 reverse at 500 counts/sec
  // delay(100);
}

void loop() {
  // Variables to store the read speed values (uint32_t)
  uint32_t speed1_u, speed2_u;
  // We will cast to int32_t for printing since speeds are signed concepts.

  // Attempt to read the speed values for both motors
  // The function returns true on success, false on failure.
  // The values are stored in speed1_u and speed2_u.
  bool success = controller.GetSpeeds(MOTOR_ADDRESS, speed1_u, speed2_u);

  if (success) {
    Serial.println("GETSPEEDS command successful.");
    Serial.println("Current Speeds:");
    Serial.print("  Motor 1 Speed: ");
    Serial.print((int32_t)speed1_u); // Cast to signed int32_t for correct representation
    Serial.print(", Motor 2 Speed: ");
    Serial.println((int32_t)speed2_u); // Cast to signed int32_t for correct representation
    Serial.println(" (counts/sec)");
  } else {
    Serial.println("GETSPEEDS command failed.");
    Serial.println("Check wiring, power, address, baud rate, and controller configuration (Are encoders enabled and Velocity PID active?).");
  }

  // Wait a short time before reading again
  delay(100); // Reading speeds frequently is common
}