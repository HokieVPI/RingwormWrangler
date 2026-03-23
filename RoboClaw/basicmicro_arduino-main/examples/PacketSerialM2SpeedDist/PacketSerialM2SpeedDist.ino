/**
 * Basicmicro Library Example: M2SPEEDDIST (42)
 *
 * Demonstrates commanding Motor 2 to move a specific distance at a specified speed.
 * This command uses the Velocity PID and encoder feedback to execute a motion profile.
 * The distance is specified in encoder counts, and the motion stops once the distance is reached.
 * Direction and absolute/relative movement are controlled by the 'flag' parameter.
 *
 * This command requires Velocity PID to be enabled and tuned for Motor 2.
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

// Define example speed and distance for Motor 2
uint32_t motor2_speed = 500;     // Speed in counts/sec
uint32_t motor2_distance = 3000; // Distance in encoder counts
uint8_t  motor2_flag = 0;        // buffer flag: 0 = buffer command, 1 = execute immediately

void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro M2SPEEDDIST Example");
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
  // You might need to send SETM2PID here or ensure it's configured in the controller's NVM.
  // Example PID values (these are just placeholders, you need values tuned for your system):
  // controller.SetM2VelocityPID(MOTOR_ADDRESS, 0.08, 0.008, 0.0008, 1200);

  // It's also often useful to reset encoders to a known position before using absolute distance
  // controller.ResetEncoders(MOTOR_ADDRESS);
  // delay(100); // Give the controller time to process reset


  Serial.print("Attempting to command Motor 2 to move ");
  Serial.print(motor2_distance);
  Serial.print(" counts at ");
  Serial.print(motor2_speed);
  Serial.println(" counts/sec (Immediate, Forward, Absolute)");
}

void loop() {
  // Attempt to send the SpeedDistance command for Motor 2
  // The function expects uint32_t for speed and distance.
  bool success = controller.SpeedDistanceM2(MOTOR_ADDRESS, motor2_speed, motor2_distance, motor2_flag);

  if (success) {
    Serial.println("M2SPEEDDIST command successful. Motor should start moving.");
    // In a real application, you would then monitor encoder position or status
    // to know when the movement is complete.
  } else {
    Serial.println("M2SPEEDDIST command failed.");
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
  if (millis() - lastCommandTime > 5000) { // Wait 5 seconds before sending the next command
      lastCommandTime = millis();

      if (movingForward) {
          // Command reverse relative movement
          motor2_speed = 500;
          motor2_distance = 3000; // Move 3000 counts backwards
          motor2_flag = 1 | (1 << 1) | (0 << 3); // bit0=1 (Backward), bit1=1 (Relative), bit3=0 (Immediate)
          Serial.print("Commanding Motor 2: Backward, Relative, ");
          Serial.print(motor2_distance); Serial.print(" counts at ");
          Serial.print(motor2_speed); Serial.println(" counts/sec");
          movingForward = false;
      } else {
          // Command forward relative movement
          motor2_speed = 500;
          motor2_distance = 3000; // Move 3000 counts forwards
          motor2_flag = 0 | (1 << 1) | (0 << 3); // bit0=0 (Forward), bit1=1 (Relative), bit3=0 (Immediate)
          Serial.print("Commanding Motor 2: Forward, Relative, ");
          Serial.print(motor2_distance); Serial.print(" counts at ");
          Serial.print(motor2_speed); Serial.println(" counts/sec");
          movingForward = true;
      }
      // Send the command again with the new parameters
      controller.SpeedDistanceM2(MOTOR_ADDRESS, motor2_speed, motor2_distance, motor2_flag);
      Serial.println("M2SPEEDDIST command sent again.");
  }


  // Short delay to prevent flooding serial during status checks (if implemented) or just waiting
  delay(50);
}