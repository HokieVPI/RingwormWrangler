/**
 * Basicmicro Library Example: SETSERIALNUMBER (96)
 *
 * Demonstrates setting the serial number string on the Basicmicro motor controller.
 * The serial number is a string (up to 36 characters) that helps identify the device.
 *
 * Note: This command sets the serial number in the controller's RAM. To make the
 * serial number permanent so it persists across power cycles, you must send a
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

// Define the serial number string to set (max 36 characters)
const char* mySerialNumber = "MyRobotController-SN12345"; // Example string

void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro SETSERIALNUMBER Example");
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

  Serial.print("Attempting to set serial number to: ");
  Serial.println(mySerialNumber);
}

void loop() {
  // Attempt to set the serial number
  // The function returns true on success, false on failure.
  bool success = controller.SetSerialNumber(MOTOR_ADDRESS, mySerialNumber);

  if (success) {
    Serial.println("SETSERIALNUMBER command successful. Serial number set in RAM.");
    Serial.println("Remember to use WRITENVM (94) to save it permanently.");
    // Optionally, read back the serial number to verify
    char readSerialNumber[49]; // Buffer for reading (max 36 + null)
    if (controller.GetSerialNumber(MOTOR_ADDRESS, readSerialNumber)) {
        Serial.print("Verified Serial Number: ");
        Serial.println(readSerialNumber); // Should match what we set
    } else {
        Serial.println("Failed to read Serial Number for verification.");
    }

    // Example of saving to NVM afterwards (optional, and causes reset!)
    /*
    Serial.println("Attempting to save serial number to NVM (WRITENVM)...");
    if (controller.WriteNVM(MOTOR_ADDRESS)) {
        Serial.println("WRITENVM command successful. Serial number saved.");
        Serial.println("Controller is now resetting. Wait a few seconds before sending more commands.");
        delay(5000); // Wait for controller to reset and boot
    } else {
        Serial.println("WRITENVM command failed. Serial number NOT saved.");
    }
    */

  } else {
    Serial.println("SETSERIALNUMBER command failed.");
    Serial.println("Check wiring, power, address, and baud rate.");
  }

  // This command is typically set once in setup or infrequently.
  // For this example, we'll add a long delay.
  delay(10000); // Wait 10 seconds before attempting again
}