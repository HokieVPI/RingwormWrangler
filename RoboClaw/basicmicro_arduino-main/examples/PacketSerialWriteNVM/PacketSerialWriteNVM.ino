/**
 * Basicmicro Library Example: WRITENVM (94)
 *
 * Demonstrates writing the controller's current configuration from RAM to
 * non-volatile memory (NVM/Flash). Settings saved to NVM will persist
 * across power cycles.
 *
 * !!! IMPORTANT: Executing this command typically causes the controller to reset.
 * The serial connection may be temporarily lost, and the controller will take
 * a few seconds to reboot and become responsive again after a successful write. !!!
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


void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro WRITENVM Example");
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

  Serial.println("!!! WARNING: Attempting to write current settings to Flash (NVM).");
  Serial.println("!!! This will make current settings permanent and RESET the controller.");
  Serial.println("Attempting to send WRITENVM command...");
}

void loop() {
  // Attempt to write the current settings from RAM to NVM
  // The function includes the necessary "magic number" if required by the command.
  // It returns true on success, false on failure.
  bool success = controller.WriteNVM(MOTOR_ADDRESS);

  if (success) {
    Serial.println("WRITENVM command successful.");
    Serial.println("Controller settings have been saved to NVM.");
    Serial.println("Controller is now resetting. Waiting for it to become responsive...");
    // Add a delay to allow the controller to reset
    delay(5000); // Wait for 5 seconds (adjust as needed for your controller's boot time)
    Serial.println("Controller should be ready for new commands.");
  } else {
    Serial.println("WRITENVM command failed.");
    Serial.println("Check wiring, power, address, and baud rate.");
    Serial.println("Ensure the controller supports this command and is not locked.");
  }

  // This command should only be executed intentionally, not in a loop.
  // The loop will pause indefinitely after the first attempt.
  while(true); // Stop execution after setup and the single command attempt
}