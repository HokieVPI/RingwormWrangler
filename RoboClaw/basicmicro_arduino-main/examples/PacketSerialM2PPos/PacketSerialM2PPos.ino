/**
 * Basicmicro Library Example: M2PPOS (126)
 *
 * Demonstrates commanding Motor 2 to move to a position specified as a
 * percentage of the configured position range.
 *
 * This command requires Position PID control to be enabled and tuned for Motor 2,
 * and encoders must be configured and connected. The percentage position is a
 * signed 16-bit value (-32768 to +32767).
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

// Define example target positions for Motor 2 as percentage (signed 16-bit)
// -32768 is minimum configured position, +32767 is maximum configured position.
int16_t target_percent_pos_1 = 16383;  // Example: ~50% of range
int16_t target_percent_pos_2 = -16383; // Example: ~-50% of range
int16_t target_percent_pos_3 = 0;      // Example: 0% (center)


// Define buffer control flag (0 = Immediate Execution, 1 = Add to Buffer)
uint8_t buffer_mode = 0; // Example: Execute command immediately

void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro M2PPOS Example");
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

  // Note: For Percentage Position commands to work, Position PID must be enabled and tuned.
  // Encoders must be configured (e.g., Quadrature mode), AND the position MIN/MAX range
  // must be set using SETM2POSPID. The controller maps the -32768 to +32767 percentage range
  // to your configured encoder MIN/MAX range.
  // You might need to send SETM2POSPID and SETM2ENCODERMODE here or ensure they are configured in NVM.
  // Example Position PID values and range (placeholders):
  // controller.SetM2PositionPID(MOTOR_ADDRESS, 0.6, 0.06, 0.006, 12000, 15, 0, 120000); // Sets min=0, max=120000 for percentage mapping

  Serial.print("Attempting to command Motor 2 to initial percentage position: ");
  Serial.print(target_percent_pos_1); Serial.println(" (out of -32768 to 32767)");
  Serial.print(" (Buffer mode: "); Serial.print(buffer_mode); Serial.println(")");
}

void loop() {
  static unsigned long lastCommandTime = 0;
  static int command_state = 0; // 0: pos1, 1: pos2, 2: pos3

  // Check controller status or use timing to know when the previous command is done
  // before sending a new one, especially when using immediate execution (buffer_mode=0).
  // For simplicity in this example, we'll just use a delay to space out commands.
  if (millis() - lastCommandTime > 5000) { // Wait 5 seconds before sending the next command
      lastCommandTime = millis();

      int16_t current_target_percent_pos;

      switch(command_state) {
          case 0:
              current_target_percent_pos = target_percent_pos_2;
              command_state = 1;
              break;
          case 1:
              current_target_percent_pos = target_percent_pos_3;
              command_state = 2;
              break;
          case 2:
              current_target_percent_pos = target_percent_pos_1;
              command_state = 0;
              break;
      }

      Serial.print("Commanding Motor 2 to percentage position: ");
      Serial.print(current_target_percent_pos); Serial.println(" (out of -32768 to 32767)");


      // Attempt to send the M2PercentPosition command
      // The position parameter expects uint16_t, so we cast our signed int16_t.
      bool success = controller.M2PercentPosition(MOTOR_ADDRESS, current_target_percent_pos, buffer_mode);

      if (success) {
        Serial.println("M2PPOS command successful. Motor should start moving.");
        // In a real application, you might monitor position error or speed to know when it's stopped.
      } else {
        Serial.println("M2PPOS command failed.");
        Serial.println("Check wiring, power, address, baud rate, and controller configuration (Is Position PID enabled and range set?).");
      }
  }

  // Short delay in the loop
  delay(50);
}