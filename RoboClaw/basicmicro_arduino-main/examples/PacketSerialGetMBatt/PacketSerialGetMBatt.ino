/**
 * Basicmicro Library Example: GETMBATT (24)
 *
 * Demonstrates reading the main battery voltage from the Basicmicro motor controller.
 * This voltage supplies power to the motor outputs.
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

  Serial.println("Basicmicro GETMBATT Example");
  Serial.print("Connecting to controller on ");
  // Print the name of the serial port being used (if possible)
  #if defined(CONTROLLER_SERIAL) && !defined(RX_PIN) // Check if using HardwareSerial and not SoftwareSerial
    Serial.println("Hardware Serial");
  #elif defined(RX_PIN) // Check if SoftwareSerial pins are defined
    Serial.println("Software Serial");
  #else
    Serial.println("Unknown Serial type");
  #endif


  // Initialize the communication serial port for the controller
  controller.begin(9600); // Ensure baud rate matches your controller
  delay(100); // Short delay to let the controller initialize after power-up or serial init

  Serial.println("Attempting to read main battery voltage...");
}

void loop() {
  // Variable to store the read voltage value and validity flag
  uint16_t mainBatteryVoltage;
  bool valid;

  // Attempt to read the main battery voltage
  // The function returns the 16-bit voltage in tenths of a volt.
  // The optional 'valid' pointer will be set to true if communication was successful.
  mainBatteryVoltage = controller.ReadMainBatteryVoltage(MOTOR_ADDRESS, &valid);

  if (valid) {
    Serial.println("GETMBATT command successful.");
    Serial.print("Main Battery Voltage: ");
    // Convert from tenths of a volt to volts (float) for printing
    Serial.print((float)mainBatteryVoltage / 10.0, 1); // Print with 1 decimal place
    Serial.println(" V");
  } else {
    Serial.println("GETMBATT command failed.");
    Serial.println("Check wiring, power, address, and baud rate.");
  }

  // Wait a few seconds before reading again
  delay(1000);
}