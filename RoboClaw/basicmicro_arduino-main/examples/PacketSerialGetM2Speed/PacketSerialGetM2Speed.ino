/**
 * Basicmicro Library Example: GETM2SPEED (19)
 *
 * Demonstrates reading the current speed value for Motor 2 from the
 * Basicmicro motor controller. This value represents the actual speed
 * of Motor 2 in encoder counts per second, typically used for velocity control feedback.
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

  Serial.println("Basicmicro GETM2SPEED Example");
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

  Serial.println("Attempting to read Motor 2 speed...");
}

void loop() {
  // Variables to store the read speed value, status, and validity flag
  uint32_t speedValue;
  uint8_t status;
  bool valid;

  // Attempt to read the speed value for Motor 2
  // The function returns the 32-bit speed in encoder counts per second.
  // The optional 'status' pointer will be filled with a direction indicator (0=Forward, 1=Reverse).
  // The optional 'valid' pointer will be set to true if communication was successful.
  speedValue = controller.ReadSpeedM2(MOTOR_ADDRESS, &status, &valid);

  if (valid) {
    Serial.println("GETM2SPEED command successful.");
    Serial.print("Motor 2 Speed: ");
    Serial.print(speedValue); // Print unsigned value
    Serial.println(" counts/sec");
    Serial.print("Motor 2 Direction Status: ");
    Serial.println(status);
    /*
       Status codes (based on library comments):
       0 = Forward motion
       1 = Reverse motion
       Note: The speed value itself is returned as an unsigned integer.
       The direction status byte indicates the sign. If status is 1 (Reverse),
       the actual signed speed would be -speedValue.
    */
  } else {
    Serial.println("GETM2SPEED command failed.");
    Serial.println("Check wiring, power, address, baud rate, and controller configuration (Is Motor 2 encoder enabled and functioning?).");
  }

  // Wait a short time before reading again to avoid flooding the serial
  delay(100);
}