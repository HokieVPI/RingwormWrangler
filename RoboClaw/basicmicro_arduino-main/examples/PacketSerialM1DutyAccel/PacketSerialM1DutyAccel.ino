/**
 * Basicmicro Library Example: M1DUTYACCEL (52)
 *
 * Demonstrates setting the PWM duty cycle for Motor 1 with controlled acceleration.
 * The motor's PWM output will ramp up or down at the specified acceleration rate
 * until it reaches the target duty cycle. This provides smooth transitions
 * between duty cycle values.
 *
 * This command bypasses PID control and directly controls the PWM output.
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

// Define example target duty cycle and acceleration for Motor 1
// Duty cycle is signed 16-bit: -32768 (full reverse) to 32767 (full forward)
int16_t motor1_target_duty = 15000; // Example: ~45% forward duty cycle
// Acceleration is a 32-bit value, units depend on controller configuration
// This usually relates to the rate of change of the 16-bit duty value per control loop tick.
// A common value is 1000-5000 for noticeable but smooth acceleration.
uint32_t motor1_accel = 2000; // Example acceleration value

void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro M1DUTYACCEL Example");
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

  Serial.print("Attempting to set Motor 1 duty: "); Serial.print(motor1_target_duty);
  Serial.print(" with accel: "); Serial.println(motor1_accel);
}

void loop() {
  // Attempt to set the PWM duty cycle for Motor 1 with acceleration
  // The function expects a uint16_t for duty, so we cast our signed int16_t.
  // This results in the correct two's complement representation being sent.
  bool success = controller.DutyAccelM1(MOTOR_ADDRESS, (uint16_t)motor1_target_duty, motor1_accel);

  if (success) {
    // Serial.println("M1DUTYACCEL command successful."); // Don't print every time in loop
  } else {
    Serial.println("M1DUTYACCEL command failed.");
    Serial.println("Check wiring, power, address, and baud rate.");
  }

  // Change the target duty cycle periodically
  static unsigned long lastChange = 0;
  if (millis() - lastChange > 4000) { // Change every 4 seconds
      lastChange = millis();
      if (motor1_target_duty > 0) {
          motor1_target_duty = -15000; // Switch to reverse
      } else {
          motor1_target_duty = 15000; // Switch to forward
      }
      Serial.print("Switching Motor 1 target duty to: ");
      Serial.println(motor1_target_duty);
      Serial.print("With accel: "); Serial.println(motor1_accel);
  }

  // Short delay to prevent flooding serial
  delay(50);
}