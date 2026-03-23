/**
 * Basicmicro Library Example: M1SPEEDACCELDECCELPOS (65)
 *
 * Demonstrates commanding Motor 1 to move to a specific position with a
 * controlled trapezoidal velocity profile, including independent acceleration,
 * cruise speed, and deceleration rates. The movement stops precisely at the
 * target position.
 *
 * This command requires Position PID control to be enabled and tuned for Motor 1.
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

// Define example motion parameters for Motor 1
uint32_t motor1_accel = 500;     // Acceleration rate in counts/sec/sec
uint32_t motor1_speed = 1000;    // Maximum cruise speed in counts/sec
uint32_t motor1_deccel = 500;    // Deceleration rate in counts/sec/sec
int32_t  motor1_target_pos = 5000; // Target position in encoder counts (signed)

// Define control flags
// buffer flag: 0 = buffer command, 1 = execute immediately
uint8_t  motor1_flag = 0;

void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro M1SPEEDACCELDECCELPOS Example");
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
  // You might need to send SETM1POSPID here or ensure it's configured in the controller's NVM.
  // Example Position PID values (placeholders):
  // controller.SetM1PositionPID(MOTOR_ADDRESS, 0.5, 0.05, 0.005, 10000, 10, 0, 100000);

  // It's often useful to reset encoders to a known position before using absolute positioning.
  // controller.ResetEncoders(MOTOR_ADDRESS);
  // delay(100); // Give controller time to process reset


  Serial.print("Attempting to command Motor 1 to position: "); Serial.print(motor1_target_pos);
  Serial.print(" with Accel/Speed/Deccel: "); Serial.print(motor1_accel); Serial.print("/"); Serial.print(motor1_speed); Serial.print("/"); Serial.println(motor1_deccel);
  Serial.print("Flags: "); Serial.println(motor1_flag, BIN);
}

void loop() {
  // Attempt to send the SpeedAccelDeccelPosition command for Motor 1
  // The position parameter expects a uint32_t, so we cast our signed int32_t.
  bool success = controller.SpeedAccelDeccelPositionM1(MOTOR_ADDRESS,
                                                         motor1_accel,
                                                         motor1_speed,
                                                         motor1_deccel,
                                                         (uint32_t)motor1_target_pos,
                                                         motor1_flag);

  if (success) {
    Serial.println("M1SPEEDACCELDECCELPOS command successful. Motor should start moving.");
    // In a real application, you would then monitor encoder position or status
    // to know when the movement is complete.
  } else {
    Serial.println("M1SPEEDACCELDECCELPOS command failed.");
    Serial.println("Check wiring, power, address, baud rate, and controller configuration (Is Position PID enabled?).");
  }

  // This command executes the motion and stops. Sending it repeatedly will
  // re-issue the command causing the motor to try and move again.
  // For a single motion, remove the loop functionality or add a very long delay.
  // For this example, we'll alternate between two target positions.
  static unsigned long lastCommandTime = 0;
  static bool targetIsPos1 = true;

  // Check controller status to see if the previous command is done before sending a new one.
  // This is a more robust way to handle sequenced movements.
  // For simplicity in this example, we'll just use a delay.
  if (millis() - lastCommandTime > 8000) { // Wait 8 seconds before sending the next command
      lastCommandTime = millis();

      if (targetIsPos1) {
          motor1_target_pos = 10000; // Move to position 10000
          motor1_flag = 0;  // 0 = buffer command, 1 = execute immediately
          Serial.print("Commanding Motor 1: Position: "); Serial.println(motor1_target_pos);
          targetIsPos1 = false;
      } else {
          motor1_target_pos = 0;   // Move back to position 0
          motor1_flag = 0;  // 0 = buffer command, 1 = execute immediately
          Serial.print("Commanding Motor 1: Position: "); Serial.println(motor1_target_pos);
          targetIsPos1 = true;
      }

      // Send the command again with the new parameters
      controller.SpeedAccelDeccelPositionM1(MOTOR_ADDRESS,
                                             motor1_accel,
                                             motor1_speed,
                                             motor1_deccel,
                                             (uint32_t)motor1_target_pos,
                                             motor1_flag);
      Serial.println("M1SPEEDACCELDECCELPOS command sent again.");
  }


  // Short delay to prevent flooding serial during status checks (if implemented) or just waiting
  delay(50);
}