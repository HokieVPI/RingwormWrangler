/**
 * Basicmicro Library Example: MIXEDSPEEDACCEL (40)
 *
 * Demonstrates setting the target speeds for both Motor 1 and Motor 2
 * with controlled acceleration in a single command. Both motors use the
 * same acceleration rate but can have different target speeds.
 *
 * This command requires Velocity PID to be enabled and tuned for both motors.
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

// Define example acceleration and target speeds for both motors
uint32_t common_accel = 300; // Common acceleration rate in counts/sec/sec
int32_t motor1_target_speed = 700; // Target speed for M1 (signed)
int32_t motor2_target_speed = -700; // Target speed for M2 (signed)

void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro MIXEDSPEEDACCEL Example");
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

  // Note: For Speed commands to work, Velocity PID must be enabled and tuned for both motors.
  // You might need to send SETM1PID and SETM2PID here or ensure it's configured in the controller's NVM.
  // Example PID values (these are just placeholders, you need values tuned for your system):
  // controller.SetM1VelocityPID(MOTOR_ADDRESS, 0.1, 0.01, 0.001, 1000);
  // controller.SetM2VelocityPID(MOTOR_ADDRESS, 0.08, 0.008, 0.0008, 1200);

  Serial.print("Attempting to set common accel: "); Serial.print(common_accel);
  Serial.print(", M1 speed: "); Serial.print(motor1_target_speed);
  Serial.print(", M2 speed: "); Serial.print(motor2_target_speed);
  Serial.println(" counts/sec");
}

void loop() {
  // Attempt to set the target speeds for both motors with a common acceleration
  // The speed parameters expect uint32_t, so we cast our signed int32_t values.
  bool success = controller.SpeedAccelM1M2(MOTOR_ADDRESS, common_accel, (uint32_t)motor1_target_speed, (uint32_t)motor2_target_speed);

  if (success) {
    // Serial.println("MIXEDSPEEDACCEL command successful."); // Don't print every time in loop
  } else {
    Serial.println("MIXEDSPEEDACCEL command failed.");
    Serial.println("Check wiring, power, address, baud rate, and controller configuration (Is Velocity PID enabled for both motors?).");
  }

  // Change the target speeds periodically
  static unsigned long lastChange = 0;
  if (millis() - lastChange > 5000) { // Change every 5 seconds
      lastChange = millis();
      // Swap directions
      int32_t temp_speed = motor1_target_speed;
      motor1_target_speed = motor2_target_speed;
      motor2_target_speed = temp_speed;

      Serial.print("Switching common accel: "); Serial.print(common_accel);
      Serial.print(", M1 speed: "); Serial.print(motor1_target_speed);
      Serial.print(", M2 speed: "); Serial.print(motor2_target_speed);
      Serial.println(" counts/sec");
  }

  // Short delay to prevent flooding serial
  delay(50);
}