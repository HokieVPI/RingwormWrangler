/**
 * Basicmicro Library Example: MIXEDPOS (121)
 *
 * Demonstrates commanding both Motor 1 and Motor 2 to move to specific
 * absolute encoder positions using the controller's default speed and
 * acceleration values in a single command.
 *
 * This command requires Position PID control to be enabled and tuned for both motors,
 * and encoders must be configured and connected. The target positions are
 * 32-bit values (signed concepts, sent as uint32_t).
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

// Define example target positions for both motors (signed 32-bit counts)
int32_t m1_target_pos_A = 5000;   // M1 First target position
int32_t m2_target_pos_A = 6000;   // M2 First target position

int32_t m1_target_pos_B = 0;      // M1 Second target position
int32_t m2_target_pos_B = 0;      // M2 Second target position


// Define buffer control flag (0 = Immediate Execution, 1 = Add to Buffer)
uint8_t buffer_mode = 0; // Example: Execute command immediately

void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro MIXEDPOS Example");
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

  // Note: For Position commands to work, Position PID must be enabled and tuned for both motors.
  // Encoders must also be configured (e.g., Quadrature mode).
  // You might need to send SETM1POSPID, SETM2POSPID, SETM1ENCODERMODE, SETM2ENCODERMODE here
  // or ensure they are configured in the controller's NVM.
  // Example Position PID values (placeholders):
  // controller.SetM1PositionPID(MOTOR_ADDRESS, 0.5, 0.05, 0.005, 10000, 10, 0, 100000);
  // controller.SetM2PositionPID(MOTOR_ADDRESS, 0.6, 0.06, 0.006, 12000, 15, 0, 120000);
  // controller.SetM1EncoderMode(MOTOR_ADDRESS, 0); // 0 for Quadrature
  // controller.SetM2EncoderMode(MOTOR_ADDRESS, 0); // 0 for Quadrature

  // It's often useful to reset encoders to a known position (e.g., 0) before using absolute positioning.
  // Serial.println("Resetting encoders to 0...");
  // controller.ResetEncoders(MOTOR_ADDRESS);
  // delay(100); // Give controller time to process reset


  Serial.print("Attempting to command Motors to initial absolute position M1: "); Serial.print(m1_target_pos_A);
  Serial.print(", M2: "); Serial.println(m2_target_pos_A);
  Serial.print(" (Buffer mode: "); Serial.print(buffer_mode); Serial.println(")");
}

void loop() {
  static unsigned long lastCommandTime = 0;
  static bool targetIsPosA = true;
  int32_t current_m1_target_pos, current_m2_target_pos;

  // Check controller status or use timing to know when the previous command is done
  // before sending a new one, especially when using immediate execution (buffer_mode=0).
  // For simplicity in this example, we'll just use a delay to space out commands.
  if (millis() - lastCommandTime > 7000) { // Wait 7 seconds before sending the next command
      lastCommandTime = millis();

      if (targetIsPosA) {
          current_m1_target_pos = m1_target_pos_B; // Move to the second target
          current_m2_target_pos = m2_target_pos_B; // Move to the second target
          targetIsPosA = false;
          Serial.print("Commanding Motors to absolute position M1: "); Serial.print(current_m1_target_pos);
          Serial.print(", M2: "); Serial.println(current_m2_target_pos);
      } else {
          current_m1_target_pos = m1_target_pos_A; // Move back to the first target
          current_m2_target_pos = m2_target_pos_A; // Move back to the first target
          targetIsPosA = true;
          Serial.print("Commanding Motors to absolute position M1: "); Serial.print(current_m1_target_pos);
          Serial.print(", M2: "); Serial.println(current_m2_target_pos);
      }

      // Attempt to send the MixedPosition command
      // The function expects uint32_t for positions, so we cast our signed int32_t values.
      bool success = controller.MixedPosition(MOTOR_ADDRESS,
                                               (uint32_t)current_m1_target_pos,
                                               (uint32_t)current_m2_target_pos,
                                               buffer_mode);

      if (success) {
        Serial.println("MIXEDPOS command successful. Motors should start moving.");
        // In a real application, you might monitor position error or speed to know when they've stopped.
      } else {
        Serial.println("MIXEDPOS command failed.");
        Serial.println("Check wiring, power, address, baud rate, and controller configuration (Is Position PID enabled for both motors?).");
      }
  }

  // Short delay in the loop
  delay(50);
}