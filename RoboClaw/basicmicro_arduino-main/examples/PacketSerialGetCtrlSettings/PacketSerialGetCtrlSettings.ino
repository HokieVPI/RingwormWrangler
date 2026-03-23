/**
 * Basicmicro Library Example: GETCTRLSETTINGS (77)
 *
 * Demonstrates reading various low-level control parameters for both motors,
 * including deadband, signal limits, and center positions. These settings
 * affect how the controller interprets input signals and applies motor power.
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

  Serial.println("Basicmicro GETCTRLSETTINGS Example");
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

  Serial.println("Attempting to read control settings...");
}

void loop() {
  // Variables to store the read control settings parameters
  uint8_t read_m1_minDB, read_m1_maxDB, read_m2_minDB, read_m2_maxDB;
  uint16_t read_m1_minLimits, read_m1_maxLimits, read_m1_center, read_m1_minVal, read_m1_maxVal;
  uint16_t read_m2_minLimits, read_m2_maxLimits, read_m2_center, read_m2_minVal, read_m2_maxVal;

  // Attempt to retrieve the control settings for both motors
  // The function returns true on success, false on failure.
  // All provided variables will be filled with the data on success.
  bool success = controller.GetCtrlSettings(MOTOR_ADDRESS,
                                             read_m1_minDB, read_m1_maxDB, read_m1_minLimits, read_m1_maxLimits, read_m1_center, read_m1_minVal, read_m1_maxVal,
                                             read_m2_minDB, read_m2_maxDB, read_m2_minLimits, read_m2_maxLimits, read_m2_center, read_m2_minVal, read_m2_maxVal);

  if (success) {
    Serial.println("GETCTRLSETTINGS command successful.");
    Serial.println("--- Current Control Settings ---");
    Serial.println("--- Motor 1 Settings ---");
    Serial.print("minDB: "); Serial.print(read_m1_minDB);
    Serial.print(", maxDB: "); Serial.print(read_m1_maxDB);
    Serial.print(", minLimits: "); Serial.print(read_m1_minLimits);
    Serial.print(", maxLimits: "); Serial.print(read_m1_maxLimits);
    Serial.print(", Center: "); Serial.print(read_m1_center);
    Serial.print(", minVal: "); Serial.print(read_m1_minVal);
    Serial.print(", maxVal: "); Serial.println(read_m1_maxVal);
    Serial.println("--- Motor 2 Settings ---");
    Serial.print("minDB: "); Serial.print(read_m2_minDB);
    Serial.print(", maxDB: "); Serial.print(read_m2_maxDB);
    Serial.print(", minLimits: "); Serial.print(read_m2_minLimits);
    Serial.print(", maxLimits: "); Serial.print(read_m2_maxLimits);
    Serial.print(", Center: "); Serial.print(read_m2_center);
    Serial.print(", minVal: "); Serial.print(read_m2_minVal);
    Serial.print(", maxVal: "); Serial.println(read_m2_maxVal);
    Serial.println("------------------------------");

  } else {
    Serial.println("GETCTRLSETTINGS command failed.");
    Serial.println("Check wiring, power, address, and baud rate.");
  }

  // Wait a few seconds before reading again
  delay(2000); // Reading settings infrequently is fine
}