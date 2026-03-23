/**
 * Basicmicro Library Example: SETESTOPLOCK (201) - MCP Only
 *
 * Demonstrates setting the emergency stop (E-Stop) lock state on MCP series
 * controllers. This determines how an active E-Stop condition is cleared.
 *
 * Valid lock states:
 * 0x00 = Hardware reset required (power cycle or dedicated reset pin)
 * 0x55 = Automatic reset (E-Stop clears automatically when cause is removed)
 * 0xAA = Software reset allowed (can be cleared via the RESETESTOP command)
 *
 * !!! IMPORTANT: This command is marked as "MCP only". It may not work on
 * Sabertooth or Kangaroo controllers. Consult your controller's documentation
 * regarding E-Stop functionality. Use caution when configuring E-Stop behavior. !!!
 *
 * Note: This command sets the state in the controller's RAM. To make it
 * permanent so it persists across power cycles, you must send a
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

// Define the desired E-Stop lock state (0x00, 0x55, or 0xAA)
uint8_t desired_lock_state = 0xAA; // Example: Allow software reset


void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro SETESTOPLOCK Example (MCP Only)");
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
  delay(100); // Short delay

  Serial.print("Attempting to set E-Stop Lock State to: 0x");
  Serial.println(desired_lock_state, HEX);
  Serial.println("NOTE: This command is for MCP controllers. Use caution.");
}

void loop() {
  // Attempt to set the E-Stop lock state
  // This command is typically only supported by MCP series controllers.
  // The function returns true on success, false on failure.
  bool success = controller.SetEStopLock(MOTOR_ADDRESS, desired_lock_state);

  if (success) {
    Serial.println("SETESTOPLOCK command successful. Lock state set in RAM.");
    Serial.println("Remember to use WRITENVM (94) to save permanently (causes reset).");
    // Optionally, read back the setting to verify (using GETESTOPLOCK)
    /*
    uint8_t read_lock_state;
    if (controller.GetEStopLock(MOTOR_ADDRESS, read_lock_state)) {
        Serial.print("Verified E-Stop Lock State: 0x");
        Serial.println(read_lock_state, HEX); // Should match what we set
    } else {
        Serial.println("Failed to read E-Stop Lock State for verification.");
    }
    */

    // Example of saving to NVM afterwards (optional, and causes reset!)
    /*
    Serial.println("Attempting to save E-Stop lock state to NVM (WRITENVM)...");
    if (controller.WriteNVM(MOTOR_ADDRESS)) {
        Serial.println("WRITENVM command successful. Lock state saved.");
        Serial.println("Controller is now resetting. Wait a few seconds before sending more commands.");
        delay(5000); // Wait for controller to reset and boot
    } else {
        Serial.println("WRITENVM command failed. Lock state NOT saved.");
    }
    */

  } else {
    Serial.println("SETESTOPLOCK command failed.");
    Serial.println("Check wiring, power, address, baud rate, controller model, and configuration.");
    Serial.println("This command may not be supported by your controller or the state value is invalid.");
  }

  // E-Stop lock state is typically set once in setup or infrequently.
  // For this example, we'll add a long delay.
  delay(10000); // Wait 10 seconds before attempting again
}