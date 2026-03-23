/**
 * Basicmicro Library Example: GETPWMIDLE (161)
 *
 * Demonstrates reading the configured PWM idle behavior for Motor 1 and Motor 2.
 * This includes the idle delay (in seconds) and the idle mode (Coast or Brake).
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

  Serial.println("Basicmicro GETPWMIDLE Example");
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

  Serial.println("Attempting to read PWM idle parameters...");
}

void loop() {
  // Variables to store the read PWM idle parameters
  float idle_delay1_sec, idle_delay2_sec; // In seconds
  bool  idle_mode1_brake, idle_mode2_brake; // true if Brake mode, false if Coast

  // Attempt to read the PWM idle parameters for both motors
  // The function returns true on success, false on failure.
  // The values are stored in the provided variables.
  bool success = controller.GetPWMIdle(MOTOR_ADDRESS,
                                       idle_delay1_sec, idle_mode1_brake,
                                       idle_delay2_sec, idle_mode2_brake);

  if (success) {
    Serial.println("GETPWMIDLE command successful.");
    Serial.println("Current PWM Idle Settings:");
    Serial.print("  Motor 1 Idle: Delay "); Serial.print(idle_delay1_sec, 1); Serial.print("s, Mode "); Serial.println(idle_mode1_brake ? "Brake" : "Coast");
    Serial.print("  Motor 2 Idle: Delay "); Serial.print(idle_delay2_sec, 1); Serial.print("s, Mode "); Serial.println(idle_mode2_brake ? "Brake" : "Coast");
  } else {
    Serial.println("GETPWMIDLE command failed.");
    Serial.println("Check wiring, power, address, and baud rate.");
  }

  // Wait a few seconds before reading again
  delay(2000); // Reading settings infrequently is fine
}