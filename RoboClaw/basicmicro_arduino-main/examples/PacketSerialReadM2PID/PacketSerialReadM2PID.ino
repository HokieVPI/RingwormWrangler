/**
 * Basicmicro Library Example: READM2PID (56)
 *
 * Demonstrates reading the current Velocity PID control parameters for Motor 2.
 * These parameters are used for closed-loop speed control.
 * The library converts the controller's internal fixed-point values to floating point.
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

  Serial.println("Basicmicro READM2PID Example");
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

  Serial.println("Attempting to read Motor 2 Velocity PID parameters...");
}

void loop() {
  // Variables to store the read PID parameters
  float read_Kp, read_Ki, read_Kd;
  uint32_t read_qpps;

  // Attempt to read the Velocity PID parameters for Motor 2
  // The function returns true on success, false on failure.
  // The parameters are stored in the provided variables.
  bool success = controller.ReadM2VelocityPID(MOTOR_ADDRESS, read_Kp, read_Ki, read_Kd, read_qpps);

  if (success) {
    Serial.println("READM2PID command successful.");
    Serial.println("Motor 2 Velocity PID Parameters:");
    Serial.print("  Kp: "); Serial.println(read_Kp, 4); // Print with 4 decimal places
    Serial.print("  Ki: "); Serial.println(read_Ki, 4);
    Serial.print("  Kd: "); Serial.println(read_Kd, 4);
    Serial.print("  MaxQPPS: "); Serial.println(read_qpps);
  } else {
    Serial.println("READM2PID command failed.");
    Serial.println("Check wiring, power, address, and baud rate.");
  }

  // Wait a few seconds before reading again
  delay(2000);
}