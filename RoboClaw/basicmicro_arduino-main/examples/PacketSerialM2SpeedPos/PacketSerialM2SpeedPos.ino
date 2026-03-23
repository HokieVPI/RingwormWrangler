/**
 * Basicmicro Library Example: M2SPEEDPOS (123)
 *
 * Demonstrates commanding Motor 2 to move to a specific absolute encoder position
 * at a specified maximum speed. The motor will accelerate/decelerate using default
 * rates (or potentially PID tuning) to reach the target speed, travel to the position,
 * and stop.
 *
 * This command requires Position PID control to be enabled and tuned for Motor 2,
 * and encoders must be configured and connected. The speed is in encoder counts
 * per second (signed concept), and the target position is a 32-bit value (signed concept).
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

// Define example motion parameters for Motor 2
int32_t motor2_speed = 900;      // Maximum speed in counts/sec (signed concept)
int32_t target_pos_1 = 6000;    // First target position (signed 32-bit counts)
int32_t target_pos_2 = -6000;   // Second target position (signed 32-bit counts)


// Define buffer control flag (0 = Immediate Execution, 1 = Add to Buffer)
uint8_t buffer_mode = 0; // Example: Execute command immediately

void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro M2SPEEDPOS Example");
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

  // Note: For Position commands to work, Position PID must be enabled and tuned.
  // Encoders must also be configured (e.g., Quadrature mode).
  // You might need to send SETM2POSPID and SETM2ENCODERMODE here
  // or ensure they are configured in the controller's NVM.
  // Example Position PID values (placeholders):
  // controller.SetM2PositionPID(MOTOR_ADDRESS, 0.6, 0.06, 0.006, 12000, 15, -12000, 12000); // Added min/max range
  // controller.SetM2EncoderMode(MOTOR_ADDRESS, 0); // 0 for Quadrature

  // It's often useful to reset encoders to a known position (e.g., 0) before using absolute positioning.
  // Serial.println("Resetting encoders to 0...");
  // controller.ResetEncoders(MOTOR_ADDRESS);
  // delay(100); // Give controller time to process reset

  Serial.print("Attempting to command Motor 2 to initial absolute position: "); Serial.print(target_pos_1);
  Serial.print(" at speed: "); Serial.print(motor2_speed);
  Serial.print(" (Buffer mode: "); Serial.print(buffer_mode); Serial.println(")");
}

void loop() {
  static unsigned long lastCommandTime = 0;
  static bool targetIsPos1 = true;
  int32_t current_target_pos;

  // Check controller status or use timing to know when the previous command is done
  // before sending a new one, especially when using immediate execution (buffer_mode=0).
  // For simplicity in this example, we'll just use a delay to space out commands.
  if (millis() - lastCommandTime > 7000) { // Wait 7 seconds before sending the next command
      lastCommandTime = millis();

      if (targetIsPos1) {
          current_target_pos = target_pos_2; // Move to the second target
          targetIsPos1 = false;
          Serial.print("Commanding Motor 2 to absolute position: "); Serial.print(current_target_pos);
          Serial.print(" at speed: "); Serial.println(motor2_speed);
      } else {
          current_target_pos = target_pos_1; // Move back to the first target
          targetIsPos1 = true;
          Serial.print("Commanding Motor 2 to absolute position: "); Serial.print(current_target_pos);
          Serial.print(" at speed: "); Serial.println(motor2_speed);
      }

      // Attempt to send the M2SpeedPosition command
      // The speed and position parameters expect uint32_t, so we cast our signed int32_t values.
      bool success = controller.M2SpeedPosition(MOTOR_ADDRESS, (uint32_t)motor2_speed, (uint32_t)current_target_pos, buffer_mode);

      if (success) {
        Serial.println("M2SPEEDPOS command successful. Motor should start moving.");
        // In a real application, you might monitor position error or speed to know when it's stopped.
      } else {
        Serial.println("M2SPEEDPOS command failed.");
        Serial.println("Check wiring, power, address, baud rate, and controller configuration (Is Position PID enabled?).");
      }
  }

  // Short delay in the loop
  delay(50);
}