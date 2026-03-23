/**
 * Basicmicro Library Example: GETTEMPS (101)
 *
 * Demonstrates reading the temperature values from both temperature sensor 1
 * and temperature sensor 2 (if available) in a single command. This is a
 * convenience command that combines the information from GETTEMP and GETTEMP2.
 *
 * The returned values are in tenths of a degree Celsius (e.g., 250 = 25.0°C).
 * Sensor 2 may not be available on all controller models.
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

  Serial.println("Basicmicro GETTEMPS Example");
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

  Serial.println("Attempting to read temperatures...");
  Serial.println("NOTE: Temperature Sensor 2 may not be available on all controller models.");
}

void loop() {
  // Variables to store the read temperature values (in tenths of a degree Celsius)
  uint16_t temp1_tenths, temp2_tenths;

  // Attempt to read the temperatures from both sensors
  // The function returns true on success, false on failure.
  // The values are stored in the provided variables.
  bool success = controller.GetTemps(MOTOR_ADDRESS, temp1_tenths, temp2_tenths);

  if (success) {
    Serial.println("GETTEMPS command successful.");
    Serial.println("Temperatures:");
    Serial.print("  Sensor 1: "); Serial.print((float)temp1_tenths / 10.0, 1); Serial.println(" C");
    Serial.print("  Sensor 2: "); Serial.print((float)temp2_tenths / 10.0, 1); Serial.println(" C");
  } else {
    Serial.println("GETTEMPS command failed.");
    Serial.println("Check wiring, power, address, and baud rate.");
  }

  // Wait a few seconds before reading again
  delay(1000); // Reading temperatures periodically is common
}