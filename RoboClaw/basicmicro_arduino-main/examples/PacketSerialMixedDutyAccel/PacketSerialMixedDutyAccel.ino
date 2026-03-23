/**
 * Basicmicro Library Example: MIXEDDUTYACCEL (54)
 *
 * Demonstrates setting the PWM duty cycles for both Motor 1 and Motor 2
 * with independent acceleration rates in a single command.
 * Each motor's PWM output will ramp up or down at its specified acceleration rate
 * until it reaches its target duty cycle.
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

// Define example target duty cycles and accelerations for both motors independently
// Duty cycle is signed 16-bit: -32768 (full reverse) to 32767 (full forward)
int16_t motor1_target_duty = 15000; // Example: ~45% forward
int16_t motor2_target_duty = -18000; // Example: ~55% reverse
// Acceleration is a 32-bit value, units depend on controller configuration
uint32_t motor1_accel = 2000; // Example acceleration for M1
uint32_t motor2_accel = 2500; // Example acceleration for M2


void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro MIXEDDUTYACCEL Example");
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

  Serial.print("Attempting to set M1 Duty/Accel: "); Serial.print(motor1_target_duty); Serial.print("/"); Serial.print(motor1_accel);
  Serial.print(", M2 Duty/Accel: "); Serial.print(motor2_target_duty); Serial.print("/"); Serial.print(motor2_accel);
  Serial.println();
}

void loop() {
  // Attempt to set the PWM duty cycles for both motors with independent acceleration
  // The function expects uint16_t for duty, so we cast our signed int16_t values.
  bool success = controller.DutyAccelM1M2(MOTOR_ADDRESS,
                                            (uint16_t)motor1_target_duty, motor1_accel,
                                            (uint16_t)motor2_target_duty, motor2_accel);

  if (success) {
    // Serial.println("MIXEDDUTYACCEL command successful."); // Don't print every time in loop
  } else {
    Serial.println("MIXEDDUTYACCEL command failed.");
    Serial.println("Check wiring, power, address, and baud rate.");
  }

  // Change the target duty cycles periodically
  static unsigned long lastChange = 0;
  if (millis() - lastChange > 5000) { // Change every 5 seconds
      lastChange = millis();
      // Swap directions and potentially values
      int16_t temp_duty1 = motor1_target_duty;
      int16_t temp_duty2 = motor2_target_duty;
      uint32_t temp_accel1 = motor1_accel;
      uint32_t temp_accel2 = motor2_accel;


      motor1_target_duty = -temp_duty1; // Reverse M1 duty
      motor2_target_duty = -temp_duty2; // Reverse M2 duty
      // Optionally swap accels as well
      // motor1_accel = temp_accel2;
      // motor2_accel = temp_accel1;


      Serial.print("Switching M1 Duty/Accel: "); Serial.print(motor1_target_duty); Serial.print("/"); Serial.print(motor1_accel);
      Serial.print(", M2 Duty/Accel: "); Serial.print(motor2_target_duty); Serial.print("/"); Serial.print(motor2_accel);
      Serial.println();
  }

  // Short delay to prevent flooding serial
  delay(50);
}