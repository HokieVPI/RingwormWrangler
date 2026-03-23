/**
 * Basicmicro Library Example: MIXEDSPEEDDIST (43)
 *
 * Demonstrates commanding both Motor 1 and Motor 2 to move specific distances
 * at specified speeds in a single command. This uses the Velocity PID and encoder
 * feedback for coordinated movement. Both motors stop once their respective distances are reached.
 * Direction and absolute/relative movement are controlled by the 'flag' parameter bits for each motor.
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

// Define example speeds and distances for both motors
uint32_t motor1_speed = 400;     // Speed in counts/sec for Motor 1
uint32_t motor1_distance = 2000; // Distance in encoder counts for Motor 1
uint32_t motor2_speed = 500;     // Speed in counts/sec for Motor 2
uint32_t motor2_distance = 3000; // Distance in encoder counts for Motor 2

// buffer flag: 0 = buffer command, 1 = execute immediately
uint8_t  control_flag = 0;

void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro MIXEDSPEEDDIST Example");
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

  Serial.print("Attempting to command M1 dist: "); Serial.print(motor1_distance); Serial.print(" at "); Serial.print(motor1_speed);
  Serial.print(", M2 dist: "); Serial.print(motor2_distance); Serial.print(" at "); Serial.print(motor2_speed);
  Serial.println(" counts/sec (Immediate, Forward, Absolute)");
}

void loop() {
  // Attempt to send the SpeedDistance command for both motors
  // The function expects uint32_t for speeds and distances.
  bool success = controller.SpeedDistanceM1M2(MOTOR_ADDRESS, motor1_speed, motor1_distance, motor2_speed, motor2_distance, control_flag);

  if (success) {
    Serial.println("MIXEDSPEEDDIST command successful. Motors should start moving.");
    // In a real application, you would then monitor encoder positions or status
    // to know when the movement is complete.
  } else {
    Serial.println("MIXEDSPEEDDIST command failed.");
    Serial.println("Check wiring, power, address, baud rate, and controller configuration (Is Velocity PID enabled?).");
  }

  // This command executes the motion and stops. Sending it repeatedly will
  // re-issue the command causing the motors to try and move again.
  // For a single coordinated motion, remove the loop functionality or add a very long delay.
  // For this example, we'll change the target distances and directions periodically.
  static unsigned long lastCommandTime = 0;
  static bool movingForward = true;

  // Check controller status to see if the previous command is done before sending a new one.
  // This is a more robust way to handle sequenced movements.
  // For simplicity in this example, we'll just use a delay.
  if (millis() - lastCommandTime > 6000) { // Wait 6 seconds before sending the next command
      lastCommandTime = millis();

      if (movingForward) {
          // Command reverse relative movement
          motor1_speed = 400;
          motor1_distance = 2000;
          motor2_speed = 500;
          motor2_distance = 3000;
          control_flag = (1 << 0) | (1 << 1) | // M1: Backward, Relative
                         (1 << 2) | (1 << 3) | // M2: Backward, Relative
                         (0 << 4);             // Immediate execution

          Serial.print("Commanding M1/M2: Backward, Relative. M1 dist: "); Serial.print(motor1_distance); Serial.print(" at "); Serial.print(motor1_speed);
          Serial.print(", M2 dist: "); Serial.print(motor2_distance); Serial.print(" at "); Serial.print(motor2_speed);
          Serial.println(" counts/sec");
          movingForward = false;
      } else {
          // Command forward relative movement
          motor1_speed = 400;
          motor1_distance = 2000;
          motor2_speed = 500;
          motor2_distance = 3000;
           control_flag = (0 << 0) | (1 << 1) | // M1: Forward, Relative
                          (0 << 2) | (1 << 3) | // M2: Forward, Relative
                          (0 << 4);             // Immediate execution
          Serial.print("Commanding M1/M2: Forward, Relative. M1 dist: "); Serial.print(motor1_distance); Serial.print(" at "); Serial.print(motor1_speed);
          Serial.print(", M2 dist: "); Serial.print(motor2_distance); Serial.print(" at "); Serial.print(motor2_speed);
          Serial.println(" counts/sec");
          movingForward = true;
      }
      // Send the command again with the new parameters
      controller.SpeedDistanceM1M2(MOTOR_ADDRESS, motor1_speed, motor1_distance, motor2_speed, motor2_distance, control_flag);
      Serial.println("MIXEDSPEEDDIST command sent again.");
  }


  // Short delay to prevent flooding serial during status checks (if implemented) or just waiting
  delay(50);
}