/**
 * Basicmicro Library Example: SETM2LR (129) - MCP Only
 *
 * Demonstrates setting the motor inductance (L) and resistance (R) parameters
 * for Motor 2 on MCP series controllers. These values are used for advanced
 * motor control algorithms like field-oriented control (FOC) or current control.
 *
 * The parameters are specified as floating-point values (Henries for L, Ohms for R).
 * The library converts these floats to their raw 32-bit representation for sending.
 *
 * !!! IMPORTANT: This command is marked as "MCP only". It may not work on
 * Sabertooth or Kangaroo controllers. Consult your controller's documentation
 * regarding the meaning of L and R values and valid ranges for your specific motor. !!!
 *
 * Note: This command sets the values in the controller's RAM. To make them
 * permanent so they persist across power cycles, you must send a
 * WRITENVM (94) command afterwards. Be aware that WRITENVM will cause the
 * controller to reset.
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

// Define example Motor 2 Inductance (L) and Resistance (R) values (floating point)
// !!! These are example placeholder values. You MUST use values
// appropriate for your specific motor, likely from its datasheet or measurement. !!!
float motor2_inductance_H = 0.0003; // Example: 300 uH
float motor2_resistance_Ohm = 0.2;  // Example: 0.2 Ohm


void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro SETM2LR Example (MCP Only)");
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

  Serial.print("Attempting to set Motor 2 L: "); Serial.print(motor2_inductance_H, 6); // Print with more precision
  Serial.print(" H, R: "); Serial.print(motor2_resistance_Ohm, 4); // Print with more precision
  Serial.println(" Ohm");
}

void loop() {
  // Attempt to set the Motor 2 L and R values
  // This command is typically only supported by MCP series controllers.
  // The function returns true on success, false on failure.
  bool success = controller.SetM2LR(MOTOR_ADDRESS, motor2_inductance_H, motor2_resistance_Ohm);

  if (success) {
    Serial.println("SETM2LR command successful. Values set in RAM.");
    Serial.println("Remember to use WRITENVM (94) to save permanently (causes reset).");
    Serial.println("NOTE: This command is for MCP controllers and parameters are motor-specific.");
    // Optionally, read back the setting to verify (using GETM2LR)
    /*
    float read_L, read_R;
    if (controller.GetM2LR(MOTOR_ADDRESS, read_L, read_R)) {
        Serial.println("Verified Motor 2 L/R:");
        Serial.print("  L: "); Serial.print(read_L, 6); Serial.println(" H");
        Serial.print("  R: "); Serial.print(read_R, 4); Serial.println(" Ohm");
    } else {
        Serial.println("Failed to read Motor 2 L/R for verification.");
    }
    */

    // Example of saving to NVM afterwards (optional, and causes reset!)
    /*
    Serial.println("Attempting to save L/R values to NVM (WRITENVM)...");
    if (controller.WriteNVM(MOTOR_ADDRESS)) {
        Serial.println("WRITENVM command successful. L/R values saved.");
        Serial.println("Controller is now resetting. Wait a few seconds before sending more commands.");
        delay(5000); // Wait for controller to reset and boot
    } else {
        Serial.println("WRITENVM command failed. L/R values NOT saved.");
    }
    */

  } else {
    Serial.println("SETM2LR command failed.");
    Serial.println("Check wiring, power, address, baud rate, controller model, and configuration.");
    Serial.println("This command may not be supported by your controller or values are out of range.");
  }

  // L/R values are typically set once in setup or on configuration changes.
  // For this example, we'll add a long delay.
  delay(10000); // Wait 10 seconds before attempting again
}