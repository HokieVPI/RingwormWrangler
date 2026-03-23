/**
 * Basicmicro Library Example: MIXEDSPEEDACCELDECCELPOS (67)
 *
 * Demonstrates commanding both Motor 1 and Motor 2 to move to specific
 * positions with independent, controlled trapezoidal velocity profiles.
 * This includes setting independent acceleration, cruise speed, and
 * deceleration rates for each motor. The movements stop precisely at the
 * target positions.
 *
 * This command requires Position PID control to be enabled and tuned for both motors.
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

// Define example motion parameters for both motors
uint32_t motor1_accel = 500;     // M1 Acceleration rate in counts/sec/sec
uint32_t motor1_speed = 1000;    // M1 Maximum cruise speed in counts/sec
uint32_t motor1_deccel = 500;    // M1 Deceleration rate in counts/sec/sec
int32_t  motor1_target_pos_A = 5000;  // M1 Target position A
int32_t  motor1_target_pos_B = -5000;  // M1 Target position B

uint32_t motor2_accel = 600;     // M2 Acceleration rate in counts/sec/sec
uint32_t motor2_speed = 1200;    // M2 Maximum cruise speed in counts/sec
uint32_t motor2_deccel = 600;    // M2 Deceleration rate in counts/sec/sec
int32_t  motor2_target_pos_A = 6000;  // M2 Target position A
int32_t  motor2_target_pos_B = -6000; // M2 Target position B


// Define control flags
// buffer flag: 0 = buffer command, 1 = execute immediately
uint8_t  buffer_flag = 0;

uint8_t target_pos = 0;

void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro MIXEDSPEEDACCELDECCELPOS Example");
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
  // You might need to send SETM1POSPID and SETM2POSPID here or ensure it's configured in the controller's NVM.
  // Example Position PID values (placeholders):
  // controller.SetM1PositionPID(MOTOR_ADDRESS, 0.5, 0.05, 0.005, 10000, 10, 0, 100000);
  // controller.SetM2PositionPID(MOTOR_ADDRESS, 0.6, 0.06, 0.006, 12000, 15, 0, 120000);

  // It's often useful to reset encoders to a known position before using absolute positioning.
   controller.ResetEncoders(MOTOR_ADDRESS);
   delay(100); // Give controller time to process reset


  Serial.print("Attempting initial command to Pos M1: "); Serial.print(motor1_target_pos_A);
  Serial.print(", M2: "); Serial.print(motor2_target_pos_A);
  Serial.println(" (Immediate)");
}

void loop() {
  static unsigned long lastCommandTime = millis() - 9500;

  // Check controller status to see if the previous command is done before sending a new one.
  // This is a more robust way to handle sequenced movements.
  // For simplicity in this example, we'll just use a delay.
  if (millis() - lastCommandTime > 10000) { // Wait 10 seconds before sending the next command
      lastCommandTime = millis();

      bool success;
      if(!target_pos){
          // Command new position
          Serial.print("Commanding Motors to Pos M1: "); Serial.print(motor1_target_pos_A);
          Serial.print(", M2: "); Serial.println(motor2_target_pos_A);

          success = controller.SpeedAccelDeccelPositionM1M2(MOTOR_ADDRESS,
                                                            motor1_accel, motor1_speed, motor1_deccel, (uint32_t)motor1_target_pos_A,
                                                            motor2_accel, motor2_speed, motor2_deccel, (uint32_t)motor2_target_pos_A,
                                                            buffer_flag);       
          target_pos=1;
      }
      else{
          // Command new position
          Serial.print("Commanding Motors to Pos M1: "); Serial.print(motor1_target_pos_B);
          Serial.print(", M2: "); Serial.println(motor2_target_pos_B);

          success = controller.SpeedAccelDeccelPositionM1M2(MOTOR_ADDRESS,
                                                            motor1_accel, motor1_speed, motor1_deccel, (uint32_t)motor1_target_pos_B,
                                                            motor2_accel, motor2_speed, motor2_deccel, (uint32_t)motor2_target_pos_B,
                                                            buffer_flag);
          target_pos=0;
      }

      if (success) {
        Serial.println("MIXEDSPEEDACCELDECCELPOS command successful. Motors should start moving.");
      } else {
        Serial.println("MIXEDSPEEDACCELDECCELPOS command failed.");
        Serial.println("Check wiring, power, address, baud rate, and controller configuration.");
      }
  }

  // Short delay in the loop
  delay(50);
}