/**
 * Basicmicro Library Example: READNVM (95) - Corrected
 *
 * Demonstrates reading the controller's configuration from non-volatile
 * memory (NVM/Flash) and loading it into RAM. This will discard any
 * unsaved changes made to the configuration in RAM.
 *
 * !!! IMPORTANT: This command loads settings from NVM into RAM and DISCARDS
 * any unsaved changes currently in RAM. !!!
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

  Serial.println("Basicmicro READNVM Example (Corrected)");
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

  Serial.println("Attempting to send READNVM command...");
}

void loop() {
  // Attempt to read the configuration from NVM into RAM
  // This will revert settings in RAM to whatever was last saved to NVM.
  // This command does NOT cause a controller reset.
  // It returns true on success, false on failure.
  bool success = controller.ReadNVM(MOTOR_ADDRESS);

  if (success) {
    Serial.println("READNVM command successful.");
    Serial.println("Controller settings have been loaded from NVM into RAM.");
    Serial.println("Any unsaved settings previously in RAM have been discarded.");
    Serial.println("You can now read settings (e.g., PID values, defaults) to see the loaded values.");
  } else {
    Serial.println("READNVM command failed.");
    Serial.println("Check wiring, power, address, and baud rate.");
    Serial.println("Ensure the controller supports this command.");
  }

  // This command is typically executed once after startup to load a desired config.
  // For this example, we'll add a long delay.
  delay(10000); // Wait 10 seconds before attempting again
}