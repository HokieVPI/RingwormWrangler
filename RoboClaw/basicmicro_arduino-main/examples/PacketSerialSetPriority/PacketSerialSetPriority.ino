/**
 * Basicmicro Library Example: SETPRIORITY (139) - MCP Only
 *
 * Demonstrates setting the priority levels for different control modes
 * (e.g., Serial, RC, Analog) on MCP series controllers. Higher priority modes
 * will override lower priority modes if multiple inputs are active.
 *
 * The specific number of priority levels and their meaning depend on the
 * specific MCP model and its configuration.
 *
 * !!! IMPORTANT: This command is marked as "MCP only". It may not work on
 * Sabertooth or Kangaroo controllers. Consult your controller's documentation
 * regarding the meaning and valid range of priority values for each level. !!!
 *
 * Note: This command sets the priorities in the controller's RAM. To make them
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

// Define example priority levels (unsigned 8-bit).
// Lower number often means higher priority, but check documentation.
// Some controllers might only support 3 priority levels as shown here.
// !!! Consult your controller's manual for the valid meaning of these values !!!
uint8_t priority1 = 10; // Example: Priority for Mode 1 (e.g., Serial)
uint8_t priority2 = 20; // Example: Priority for Mode 2 (e.g., RC)
uint8_t priority3 = 30; // Example: Priority for Mode 3 (e.g., Analog)


void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro SETPRIORITY Example (MCP Only)");
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

  Serial.print("Attempting to set Priority Levels: "); Serial.print(priority1);
  Serial.print(", "); Serial.print(priority2);
  Serial.print(", "); Serial.println(priority3);
  Serial.println("NOTE: This command is for MCP controllers and meanings are model-specific.");
}

void loop() {
  // Attempt to set the priority levels
  // This command is typically only supported by MCP series controllers.
  // The function returns true on success, false on failure.
  bool success = controller.SetPriority(MOTOR_ADDRESS, priority1, priority2, priority3);

  if (success) {
    Serial.println("SETPRIORITY command successful. Priorities set in RAM.");
    Serial.println("Remember to use WRITENVM (94) to save permanently (causes reset).");
    // Optionally, read back the setting to verify (using GETPRIORITY)
    /*
    uint8_t read_p1, read_p2, read_p3;
    if (controller.GetPriority(MOTOR_ADDRESS, read_p1, read_p2, read_p3)) {
        Serial.println("Verified Priority Levels:");
        Serial.print("  P1: "); Serial.print(read_p1);
        Serial.print(", P2: "); Serial.print(read_p2);
        Serial.print(", P3: "); Serial.println(read_p3);
    } else {
        Serial.println("Failed to read Priority Levels for verification.");
    }
    */

    // Example of saving to NVM afterwards (optional, and causes reset!)
    /*
    Serial.println("Attempting to save priorities to NVM (WRITENVM)...");
    if (controller.WriteNVM(MOTOR_ADDRESS)) {
        Serial.println("WRITENVM command successful. Priorities saved.");
        Serial.println("Controller is now resetting. Wait a few seconds before sending more commands.");
        delay(5000); // Wait for controller to reset and boot
    } else {
        Serial.println("WRITENVM command failed. Priorities NOT saved.");
    }
    */

  } else {
    Serial.println("SETPRIORITY command failed.");
    Serial.println("Check wiring, power, address, baud rate, controller model, and configuration.");
    Serial.println("This command may not be supported by your controller or parameters are invalid.");
  }

  // Priority settings are typically set once in setup or on configuration changes.
  // For this example, we'll add a long delay.
  delay(10000); // Wait 10 seconds before attempting again
}