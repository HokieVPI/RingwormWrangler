/**
 * Basicmicro Library Example: GETPOSERRORS (114)
 *
 * Demonstrates reading the current actual position error values for both
 * Motor 1 and Motor 2 while in Position PID control. The position error
 * is the difference between the target position setpoint and the actual
 * measured encoder position.
 *
 * The returned values are unsigned 16-bit integers, representing the magnitude
 * of the position error in encoder counts. A large value indicates the motor
 * is not holding the target position accurately.
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

  Serial.println("Basicmicro GETPOSERRORS Example");
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

  Serial.println("Attempting to read position error values...");

  // Example: Command motors to a position to generate a position error if not perfectly held
  // Assuming Position PID and encoders are set up, command a position:
  // controller.M1Position(MOTOR_ADDRESS, 1000, 0); // Move M1 to position 1000 (Immediate)
  // delay(100);
}

void loop() {
  // Variables to store the read position error values (unsigned 16-bit)
  uint16_t error1, error2;

  // Attempt to read the position error values for both motors
  // The function returns true on success, false on failure.
  // The values are stored in error1 and error2.
  bool success = controller.GetPosErrors(MOTOR_ADDRESS, error1, error2);

  if (success) {
    Serial.println("GETPOSERRORS command successful.");
    Serial.println("Current Position Errors:");
    Serial.print("  Motor 1 Error: "); Serial.print(error1);
    Serial.print(", Motor 2 Error: "); Serial.println(error2);
    Serial.println(" (encoder counts)");
    // If these values are consistently high when trying to hold a position,
    // it indicates tuning issues or physical limitations.
  } else {
    Serial.println("GETPOSERRORS command failed.");
    Serial.println("Check wiring, power, address, baud rate, and controller configuration (Is Position PID active?).");
  }

  // Wait a short time before reading again
  delay(100); // Reading errors periodically is common for monitoring
}