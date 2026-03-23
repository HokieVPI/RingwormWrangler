/**
 * Basicmicro Library Example: M1SPEEDACCELDIST (44)
 *
 * Demonstrates commanding Motor 1 to move a specific distance with controlled
 * acceleration and at a specified top speed. This uses the Velocity PID and
 * encoder feedback to execute a trapezoidal velocity profile.
 * Direction and absolute/relative movement are controlled by the 'flag' parameter.
 *
 * This command requires Velocity PID to be enabled and tuned for Motor 1.
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

// Define example acceleration, speed, and distance for Motor 1
uint32_t motor1_accel = 500;     // Acceleration rate in counts/sec/sec
uint32_t motor1_speed = 1000;    // Top speed in counts/sec
uint32_t motor1_distance = 5000; // Distance in encoder counts
uint8_t  motor1_flag = 0;        // buffer flag: 0 = buffer command, 1 = execute immediately

void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro M1SPEEDACCELDIST Example");
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

  // Note: For Speed/Distance commands to work, Velocity PID must be enabled and tuned.
  // You might need to send SETM1PID here or ensure it's configured in the controller's NVM.
  // Example PID values (these are just placeholders, you need values tuned for your system):
  // controller.SetM1VelocityPID(MOTOR_ADDRESS, 0.1, 0.01, 0.001, 1000);

  // It's also often useful to reset encoders to a known position before using absolute distance
  // controller.ResetEncoders(MOTOR_ADDRESS);
  // delay(100); // Give the controller time to process reset


  Serial.print("Attempting to command Motor 1 to move ");
  Serial.print(motor1_distance);
  Serial.print(" counts with accel ");
  Serial.print(motor1_accel);
  Serial.print(" to speed ");
  Serial.print(motor1_speed);
  Serial.println(" (Immediate, Forward, Absolute)");
}

void loop() {
  // Attempt to send the SpeedAccelDistance command for Motor 1
  // The function expects uint32_t for accel, speed, and distance.
  bool success = controller.SpeedAccelDistanceM1(MOTOR_ADDRESS, motor1_accel, motor1_speed, motor1_distance, motor1_flag);

  if (success) {
    Serial.println("M1SPEEDACCELDIST command successful. Motor should start moving.");
    // In a real application, you would then monitor encoder position or status
    // to know when the movement is complete.
  } else {
    Serial.println("M1SPEEDACCELDIST command failed.");
    Serial.println("Check wiring, power, address, baud rate, and controller configuration (Is Velocity PID enabled?).");
  }

  // This command executes the motion and stops. Sending it repeatedly will
  // re-issue the command causing the motor to try and move again.
  // For a single motion, remove the loop functionality or add a very long delay.
  // For this example, we'll change the target distance and direction periodically.
  static unsigned long lastCommandTime = 0;
  static bool movingForward = true;

  // Check controller status to see if the previous command is done before sending a new one.
  // This is a more robust way to handle sequenced movements.
  // For simplicity in this example, we'll just use a delay.
  if (millis() - lastCommandTime > 7000) { // Wait 7 seconds before sending the next command
      lastCommandTime = millis();

      if (movingForward) {
          // Command reverse relative movement
          motor1_accel = 500;
          motor1_speed = 1000;
          motor1_distance = 5000; // Move 5000 counts backwards
          motor1_flag = 1 | (1 << 1) | (0 << 2) | (0 << 3); // bit0=1 (Backward), bit1=1 (Relative), bit2=0 (Accel not limited), bit3=0 (Immediate)
          Serial.print("Commanding Motor 1: Backward, Relative. Accel: "); Serial.print(motor1_accel);
          Serial.print(", Speed: "); Serial.print(motor1_speed);
          Serial.print(", Distance: "); Serial.println(motor1_distance);
          movingForward = false;
      } else {
          // Command forward relative movement
          motor1_accel = 500;
          motor1_speed = 1000;
          motor1_distance = 5000; // Move 5000 counts forwards
          motor1_flag = 0 | (1 << 1) | (0 << 2) | (0 << 3); // bit0=0 (Forward), bit1=1 (Relative), bit2=0 (Accel not limited), bit3=0 (Immediate)
          Serial.print("Commanding Motor 1: Forward, Relative. Accel: "); Serial.print(motor1_accel);
          Serial.print(", Speed: "); Serial.print(motor1_speed);
          Serial.print(", Distance: "); Serial.println(motor1_distance);
          movingForward = true;
      }
      // Send the command again with the new parameters
      controller.SpeedAccelDistanceM1(MOTOR_ADDRESS, motor1_accel, motor1_speed, motor1_distance, motor1_flag);
      Serial.println("M1SPEEDACCELDIST command sent again.");
  }


  // Short delay to prevent flooding serial during status checks (if implemented) or just waiting
  delay(50);
}