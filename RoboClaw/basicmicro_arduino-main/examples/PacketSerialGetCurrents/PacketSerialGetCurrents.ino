/**
 * Basicmicro Library Example: GETCURRENTS (49)
 *
 * Demonstrates reading the current draw for Motor 1 and Motor 2 in milliamps.
 * This is useful for monitoring motor load and detecting potential issues
 * like stalls or overcurrent conditions.
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

  Serial.println("Basicmicro GETCURRENTS Example");
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

  Serial.println("Attempting to read motor current draw...");

  // Example: Command motors to move to see current draw
  // Assuming Velocity PID and encoders are set up, command a speed:
  // controller.SpeedM1M2(MOTOR_ADDRESS, 500, 500); // Move both motors forward at 500 counts/sec
  // Or command a duty cycle:
  // controller.DutyM1M2(MOTOR_ADDRESS, 10000, 10000); // Apply moderate duty cycle
  // delay(100);
}

void loop() {
  // Variables to store the read current values in milliamps
  int16_t current1_mA, current2_mA;

  // Attempt to read the current draw for both motors
  // The function returns true on success, false on failure.
  // The current values are stored in current1_mA and current2_mA.
  bool success = controller.ReadCurrents(MOTOR_ADDRESS, current1_mA, current2_mA);

  if (success) {
    Serial.println("GETCURRENTS command successful.");
    Serial.print("Motor 1 Current: ");
    Serial.print(current1_mA);
    Serial.print(" mA, Motor 2 Current: ");
    Serial.print(current2_mA);
    Serial.println(" mA");
    // Positive current indicates driving, negative indicates regenerative braking.
  } else {
    Serial.println("GETCURRENTS command failed.");
    Serial.println("Check wiring, power, address, and baud rate.");
  }

  // Wait a short time before reading again
  delay(200); // Reading current rapidly might not show significant changes
}