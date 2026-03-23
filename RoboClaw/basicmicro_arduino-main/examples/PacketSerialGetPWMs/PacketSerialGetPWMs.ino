/**
 * Basicmicro Library Example: GETPWMS (48)
 *
 * Demonstrates reading the current PWM values being sent to Motor 1 and Motor 2.
 * These values are signed 16-bit integers (-32768 to 32767) representing the duty cycle.
 * Reading these values can help monitor the output of the controller regardless
 * of the control mode (Duty, Speed, Position).
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

  Serial.println("Basicmicro GETPWMS Example");
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

  Serial.println("Attempting to read current PWM values...");

  // Example: Set a duty cycle for motors to see the PWM values change
  // controller.DutyM1M2(MOTOR_ADDRESS, 10000, -15000); // Example: Set duty cycles
  // delay(100);
}

void loop() {
  // Variables to store the read PWM values
  int16_t pwm1, pwm2;

  // Attempt to read the current PWM values for both motors
  // The function returns true on success, false on failure.
  // The PWM values are stored in pwm1 and pwm2.
  bool success = controller.ReadPWMs(MOTOR_ADDRESS, pwm1, pwm2);

  if (success) {
    Serial.println("GETPWMS command successful.");
    Serial.print("Motor 1 PWM: ");
    Serial.print(pwm1);
    Serial.print(", Motor 2 PWM: ");
    Serial.println(pwm2);
    // Values range from -32768 to 32767
  } else {
    Serial.println("GETPWMS command failed.");
    Serial.println("Check wiring, power, address, and baud rate.");
  }

  // Wait a short time before reading again
  delay(100);
}