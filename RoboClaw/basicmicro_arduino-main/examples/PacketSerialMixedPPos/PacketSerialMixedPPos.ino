/**
 * Basicmicro Library Example: MIXEDPPOS (127)
 *
 * Demonstrates commanding both Motor 1 and Motor 2 to move to positions
 * specified as percentages of their configured position ranges, in a single command.
 *
 * This command requires Position PID control to be enabled and tuned for both motors,
 * and encoders must be configured and connected. The percentage positions are
 * signed 16-bit values (-32768 to +32767).
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

// Define example target positions for both motors as percentage (signed 16-bit)
// -32768 is minimum configured position, +32767 is maximum configured position.
int16_t m1_target_percent_A = 16383;  // M1 Example: ~50% of range
int16_t m2_target_percent_A = -16383; // M2 Example: ~-50% of range

int16_t m1_target_percent_B = -16383; // M1 Example: ~-50% of range
int16_t m2_target_percent_B = 16383;  // M2 Example: ~50% of range

int16_t m1_target_percent_C = 0;      // M1 Example: 0% (center)
int16_t m2_target_percent_C = 0;      // M2 Example: 0% (center)


// Define buffer control flag (0 = Immediate Execution, 1 = Add to Buffer)
uint8_t buffer_mode = 0; // Example: Execute command immediately

void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro MIXEDPPOS Example");
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

  // Note: For Percentage Position commands to work, Position PID must be enabled and tuned for both motors.
  // Encoders must be configured (e.g., Quadrature mode), AND the position MIN/MAX ranges
  // must be set using SETM1POSPID and SETM2POSPID. The controller maps the
  // -32768 to +32767 percentage range to your configured encoder MIN/MAX ranges.
  // You might need to send SETM1POSPID, SETM2POSPID, SETM1ENCODERMODE, SETM2ENCODERMODE here
  // or ensure they are configured in NVM.
  // Example Position PID values and ranges (placeholders):
  // controller.SetM1PositionPID(MOTOR_ADDRESS, 0.5, 0.05, 0.005, 10000, 10, 0, 100000); // Sets M1 min/max range
  // controller.SetM2PositionPID(MOTOR_ADDRESS, 0.6, 0.06, 0.006, 12000, 15, -50000, 50000); // Sets M2 min/max range

  Serial.print("Attempting to command Motors to initial percentage positions M1: "); Serial.print(m1_target_percent_A);
  Serial.print(", M2: "); Serial.print(m2_target_percent_A);
  Serial.println(" (out of -32768 to 32767)");
  Serial.print(" (Buffer mode: "); Serial.print(buffer_mode); Serial.println(")");
}

void loop() {
  static unsigned long lastCommandTime = 0;
  static int command_state = 0; // 0: pos set A, 1: pos set B, 2: pos set C

  // Check controller status or use timing to know when the previous command is done
  // before sending a new one, especially when using immediate execution (buffer_mode=0).
  // For simplicity in this example, we'll just use a delay to space out commands.
  if (millis() - lastCommandTime > 6000) { // Wait 6 seconds before sending the next command
      lastCommandTime = millis();

      int16_t current_m1_target_percent, current_m2_target_percent;

      switch(command_state) {
          case 0:
              current_m1_target_percent = m1_target_percent_B;
              current_m2_target_percent = m2_target_percent_B;
              command_state = 1;
              break;
          case 1:
              current_m1_target_percent = m1_target_percent_C;
              current_m2_target_percent = m2_target_percent_C;
              command_state = 2;
              break;
          case 2:
              current_m1_target_percent = m1_target_percent_A;
              current_m2_target_percent = m2_target_percent_A;
              command_state = 0;
              break;
      }

      Serial.print("Commanding Motors to percentage positions M1: "); Serial.print(current_m1_target_percent);
      Serial.print(", M2: "); Serial.print(current_m2_target_percent);
      Serial.println(" (out of -32768 to 32767)");


      // Attempt to send the MixedPercentPosition command
      // The position parameters expect uint16_t, so we cast our signed int16_t values.
      bool success = controller.MixedPercentPosition(MOTOR_ADDRESS, current_m1_target_percent, current_m2_target_percent, buffer_mode);

      if (success) {
        Serial.println("MIXEDPPOS command successful. Motors should start moving.");
        // In a real application, you might monitor position error or speed to know when they've stopped.
      } else {
        Serial.println("MIXEDPPOS command failed.");
        Serial.println("Check wiring, power, address, baud rate, and controller configuration (Is Position PID enabled and range set for both motors?).");
      }
  }

  // Short delay in the loop
  delay(50);
}