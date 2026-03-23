/**
 * Basicmicro Library Example: MIXEDSPEEDACCELDIST (46)
 *
 * Demonstrates commanding both Motor 1 and Motor 2 to move specific distances
 * with controlled acceleration and at specified top speeds. This uses the Velocity
 * PID and encoder feedback to execute independent trapezoidal velocity profiles
 * for each motor. Both motors stop once their respective distances are reached.
 * Direction and absolute/relative movement are controlled by the 'flag' parameter bits.
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

// Define example acceleration, speed, and distance for both motors
uint32_t motor_accel = 500;     // M1 Acceleration rate in counts/sec/sec

uint32_t motor1_speed = 1000;    // M1 Top speed in counts/sec
uint32_t motor1_distance = 5000; // M1 Distance in encoder counts

uint32_t motor2_speed = 1200;    // M2 Top speed in counts/sec
uint32_t motor2_distance = 6000; // M2 Distance in encoder counts


// Define control flags
// buffer flag: 0 = buffer command, 1 = execute immediately
uint8_t  control_flag = 0;

void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro MIXEDSPEEDACCELDIST Example");
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

  // Note: For Speed/Distance commands to work, Velocity PID must be enabled and tuned for both motors.
  // You might need to send SETM1PID and SETM2PID here or ensure it's configured in the controller's NVM.
  // Example PID values (these are just placeholders, you need values tuned for your system):
  // controller.SetM1VelocityPID(MOTOR_ADDRESS, 0.1, 0.01, 0.001, 1000);
  // controller.SetM2VelocityPID(MOTOR_ADDRESS, 0.08, 0.008, 0.0008, 1200);

  // It's also often useful to reset encoders to a known position before using absolute distance
  // controller.ResetEncoders(MOTOR_ADDRESS);
  // delay(100); // Give the controller time to process reset


  Serial.print("Attempting to command M1 Accel/Speed/Dist: "); Serial.print(motor_accel); Serial.print("/"); Serial.print(motor1_speed); Serial.print("/"); Serial.print(motor1_distance);
  Serial.print("/"); Serial.print(motor2_speed); Serial.print("/"); Serial.print(motor2_distance);
  Serial.println(" (Immediate, Forward, Absolute)");
}

void loop() {
  // Attempt to send the SpeedAccelDistance command for both motors
  // The function expects uint32_t for accel, speed, and distance for each motor.
  bool success = controller.SpeedAccelDistanceM1M2(MOTOR_ADDRESS, motor_accel, 
                                                     motor1_speed, motor1_distance,
                                                     motor2_speed, motor2_distance,
                                                     control_flag);

  if (success) {
    Serial.println("MIXEDSPEEDACCELDIST command successful. Motors should start moving.");
    // In a real application, you would then monitor encoder positions or status
    // to know when the movement is complete.
  } else {
    Serial.println("MIXEDSPEEDACCELDIST command failed.");
    Serial.println("Check wiring, power, address, baud rate, and controller configuration (Is Velocity PID enabled?).");
  }

  // This command executes the motion and stops. Sending it repeatedly will
  // re-issue the command causing the motors to try and move again.
  // For a single coordinated motion, remove the loop functionality or add a very long delay.
  // For this example, we'll change the target parameters and directions periodically.
  static unsigned long lastCommandTime = 0;
  static bool movingForward = true;

  // Check controller status to see if the previous command is done before sending a new one.
  // This is a more robust way to handle sequenced movements.
  // For simplicity in this example, we'll just use a delay.
  if (millis() - lastCommandTime > 8000) { // Wait 8 seconds before sending the next command
      lastCommandTime = millis();

      if (movingForward) {
          // Command reverse relative movement with different parameters
          motor_accel = 400; 
          motor1_speed = 800; motor1_distance = 4000;
          motor2_speed = 1000; motor2_distance = 5000;

          control_flag = (1 << 0) | (1 << 1) | (0 << 2) | // M1: Backward, Relative, Accel unlimited
                         (1 << 4) | (1 << 5) | (0 << 6) | // M2: Backward, Relative, Accel unlimited
                         (0 << 3);             // Immediate execution

          Serial.print("Commanding M1/M2: Forward, Relative. M1 "); Serial.print(motor1_speed); Serial.print("/"); Serial.print(motor1_distance);
          Serial.print(", M2 "); Serial.print(motor2_speed); Serial.print("/"); Serial.print(motor2_distance);
          Serial.println();
          movingForward = false;
      } else {
          // Command forward relative movement with original parameters
          motor_accel = 500; 
          motor1_speed = 1000; motor1_distance = 5000;
          motor2_speed = 1200; motor2_distance = 6000;

           control_flag = (0 << 0) | (1 << 1) | (0 << 2) | // M1: Forward, Relative, Accel unlimited
                          (0 << 4) | (1 << 5) | (0 << 6) | // M2: Forward, Relative, Accel unlimited
                          (0 << 3);             // Immediate execution

          Serial.print("Commanding M1/M2: Forward, Relative. M1 "); Serial.print(motor1_speed); Serial.print("/"); Serial.print(motor1_distance);
          Serial.print(", M2 "); Serial.print(motor2_speed); Serial.print("/"); Serial.print(motor2_distance);
          Serial.println();
          movingForward = true;
      }
      // Send the command again with the new parameters
      controller.SpeedAccelDistanceM1M2(MOTOR_ADDRESS,
                                         motor_accel, 
                                         motor1_speed, motor1_distance,
                                         motor2_speed, motor2_distance,
                                         control_flag);
      Serial.println("MIXEDSPEEDACCELDIST command sent again.");
  }


  // Short delay to prevent flooding serial during status checks (if implemented) or just waiting
  delay(50);
}