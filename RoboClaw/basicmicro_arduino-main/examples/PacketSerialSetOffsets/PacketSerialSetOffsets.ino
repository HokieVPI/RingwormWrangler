/**
 * Basicmicro Library Example: SETOFFSETS (115) - MCP Only
 *
 * Demonstrates setting offset values for both encoders on MCP series controllers.
 * These offsets are typically used to fine-tune encoder readings or align
 * multiple axes. The meaning and application of these offsets depend on the
 * specific MCP model and configuration.
 *
 * !!! IMPORTANT: This command is marked as "MCP only". It may not work on
 * Sabertooth or Kangaroo controllers. Consult your controller's documentation. !!!
 *
 * Note: This command sets the offsets in the controller's RAM. To make them
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

// Define example offset values (unsigned 8-bit, 0-255)
// !!! Consult your controller's manual for the meaning of these values !!!
uint8_t encoder1_offset = 5; // Example offset for Encoder 1
uint8_t encoder2_offset = 10; // Example offset for Encoder 2


void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro SETOFFSETS Example (MCP Only)");
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

  Serial.print("Attempting to set Encoder 1 Offset to: "); Serial.print(encoder1_offset);
  Serial.print(", Encoder 2 Offset to: "); Serial.println(encoder2_offset);
}

void loop() {
  // Attempt to set the encoder offset values
  // This command is typically only supported by MCP series controllers.
  // The function returns true on success, false on failure.
  bool success = controller.SetOffsets(MOTOR_ADDRESS, encoder1_offset, encoder2_offset);

  if (success) {
    Serial.println("SETOFFSETS command successful. Offsets set in RAM.");
    Serial.println("Remember to use WRITENVM (94) to save permanently (causes reset).");
    // Optionally, read back the setting to verify (using GETOFFSETS)
    /*
    uint8_t read_offset1, read_offset2;
    if (controller.GetOffsets(MOTOR_ADDRESS, read_offset1, read_offset2)) {
        Serial.println("Verified Encoder Offsets:");
        Serial.print("  Encoder 1 Offset: "); Serial.println(read_offset1);
        Serial.print(", Encoder 2 Offset: "); Serial.println(read_offset2);
    } else {
        Serial.println("Failed to read Encoder Offsets for verification.");
    }
    */

    // Example of saving to NVM afterwards (optional, and causes reset!)
    /*
    Serial.println("Attempting to save offsets to NVM (WRITENVM)...");
    if (controller.WriteNVM(MOTOR_ADDRESS)) {
        Serial.println("WRITENVM command successful. Offsets saved.");
        Serial.println("Controller is now resetting. Wait a few seconds before sending more commands.");
        delay(5000); // Wait for controller to reset and boot
    } else {
        Serial.println("WRITENVM command failed. Offsets NOT saved.");
    }
    */

  } else {
    Serial.println("SETOFFSETS command failed.");
    Serial.println("Check wiring, power, address, baud rate, controller model, and configuration.");
    Serial.println("This command may not be supported by your controller.");
  }

  // Encoder offsets are typically set once in setup or on configuration changes.
  // For this example, we'll add a long delay.
  delay(10000); // Wait 10 seconds before attempting again
}