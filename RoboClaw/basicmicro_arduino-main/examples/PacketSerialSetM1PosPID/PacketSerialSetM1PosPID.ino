/**
 * Basicmicro Library Example: SETM1POSPID (61)
 *
 * Demonstrates setting the Position PID control parameters for Motor 1.
 * PID control is used for closed-loop position control based on encoder feedback.
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

// Define example PID parameters for Motor 1 Position control
// These values will need tuning for your specific motor and encoder setup.
// Position PID uses a different fixed-point format (10.14) than Velocity PID (16.16).
float m1_pos_Kp = 0.5;     // Proportional gain
float m1_pos_Ki = 0.05;    // Integral gain
float m1_pos_Kd = 0.005;   // Derivative gain

// Define other Position PID settings
uint32_t m1_pos_KiMax = 10000; // Maximum integral windup limit
uint32_t m1_pos_DeadZone = 10;  // Error deadzone (in encoder counts)
// Define the allowed range of positions. Controller will limit movement within this range.
uint32_t m1_pos_Min = 0;        // Minimum allowed encoder position
uint32_t m1_pos_Max = 100000;   // Maximum allowed encoder position

void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro SETM1POSPID Example");
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
  // You might also need to set the encoder mode (e.g. to Quadrature).
  // Ensure Motor 1 encoder is configured correctly on the controller.

  Serial.println("Attempting to set Motor 1 Position PID parameters...");
  Serial.print("Kp: "); Serial.print(m1_pos_Kp);
  Serial.print(", Ki: "); Serial.print(m1_pos_Ki);
  Serial.print(", Kd: "); Serial.print(m1_pos_Kd);
  Serial.print(", KiMax: "); Serial.print(m1_pos_KiMax);
  Serial.print(", DeadZone: "); Serial.print(m1_pos_DeadZone);
  Serial.print(", MinPos: "); Serial.print(m1_pos_Min);
  Serial.print(", MaxPos: "); Serial.println(m1_pos_Max);
}

void loop() {
  // Attempt to set the Position PID parameters for Motor 1
  // The function handles converting the float PID values to the controller's fixed-point format.
  // It returns true on success, false on failure.
  bool success = controller.SetM1PositionPID(MOTOR_ADDRESS,
                                             m1_pos_Kp, m1_pos_Ki, m1_pos_Kd,
                                             m1_pos_KiMax, m1_pos_DeadZone,
                                             m1_pos_Min, m1_pos_Max);

  if (success) {
    Serial.println("SETM1POSPID command successful.");
    // You might want to verify the settings by reading them back
    float read_Kp, read_Ki, read_Kd;
    uint32_t read_KiMax, read_DeadZone, read_Min, read_Max;
    if (controller.ReadM1PositionPID(MOTOR_ADDRESS, read_Kp, read_Ki, read_Kd, read_KiMax, read_DeadZone, read_Min, read_Max)) {
        Serial.println("Verified Motor 1 Position PID:");
        Serial.print("  Kp: "); Serial.println(read_Kp, 4); // Print with 4 decimal places
        Serial.print("  Ki: "); Serial.println(read_Ki, 4);
        Serial.print("  Kd: "); Serial.println(read_Kd, 4);
        Serial.print("  KiMax: "); Serial.println(read_KiMax);
        Serial.print("  DeadZone: "); Serial.println(read_DeadZone);
        Serial.print("  MinPos: "); Serial.println(read_Min);
        Serial.print("  MaxPos: "); Serial.println(read_Max);
    } else {
        Serial.println("Failed to read Motor 1 Position PID for verification.");
    }
  } else {
    Serial.println("SETM1POSPID command failed.");
    Serial.println("Check wiring, power, address, baud rate, and controller configuration.");
  }

  // PID settings are typically set once in setup or on configuration changes.
  // For this example, we'll add a long delay.
  delay(10000); // Wait 10 seconds before attempting again
}