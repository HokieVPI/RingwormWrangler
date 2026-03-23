/**
 * Basicmicro Library Example: SETCTRLSETTINGS (76)
 *
 * Demonstrates setting various low-level control parameters for both motors,
 * including deadband, signal limits, and center positions. These settings
 * affect how the controller interprets input signals and applies motor power.
 *
 * Consult your controller's documentation for the specific meaning and valid
 * ranges for these parameters based on the control mode being used.
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

// Define example control settings parameters for both motors.
// !!! IMPORTANT: These values are examples. You MUST consult your controller's
// manual to find the correct values for your desired control behavior.
// Incorrect values may cause unexpected behavior or unsafe operation. !!!

// Motor 1 Settings
uint8_t m1_minDB = 10;      // Minimum Deadband (8-bit)
uint8_t m1_maxDB = 10;      // Maximum Deadband (8-bit)
uint16_t m1_minLimits = 100; // Minimum Limits (16-bit)
uint16_t m1_maxLimits = 100; // Maximum Limits (16-bit)
uint16_t m1_center = 1500;   // Center (16-bit, often for RC PWM input)
uint16_t m1_minVal = 1000;   // Minimum Value (16-bit, often for RC PWM input)
uint16_t m1_maxVal = 2000;   // Maximum Value (16-bit, often for RC PWM input)

// Motor 2 Settings (Example values, adjust as needed)
uint8_t m2_minDB = 10;      // Minimum Deadband (8-bit)
uint8_t m2_maxDB = 10;      // Maximum Deadband (8-bit)
uint16_t m2_minLimits = 100; // Minimum Limits (16-bit)
uint16_t m2_maxLimits = 100; // Maximum Limits (16-bit)
uint16_t m2_center = 1500;   // Center (16-bit, often for RC PWM input)
uint16_t m2_minVal = 1000;   // Minimum Value (16-bit, often for RC PWM input)
uint16_t m2_maxVal = 2000;   // Maximum Value (16-bit, often for RC PWM input)


void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro SETCTRLSETTINGS Example");
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

  Serial.println("Attempting to set control settings...");
  Serial.println("--- Motor 1 Settings ---");
  Serial.print("minDB: "); Serial.print(m1_minDB);
  Serial.print(", maxDB: "); Serial.print(m1_maxDB);
  Serial.print(", minLimits: "); Serial.print(m1_minLimits);
  Serial.print(", maxLimits: "); Serial.print(m1_maxLimits);
  Serial.print(", Center: "); Serial.print(m1_center);
  Serial.print(", minVal: "); Serial.print(m1_minVal);
  Serial.print(", maxVal: "); Serial.println(m1_maxVal);
  Serial.println("--- Motor 2 Settings ---");
  Serial.print("minDB: "); Serial.print(m2_minDB);
  Serial.print(", maxDB: "); Serial.print(m2_maxDB);
  Serial.print(", minLimits: "); Serial.print(m2_minLimits);
  Serial.print(", maxLimits: "); Serial.print(m2_maxLimits);
  Serial.print(", Center: "); Serial.print(m2_center);
  Serial.print(", minVal: "); Serial.print(m2_minVal);
  Serial.print(", maxVal: "); Serial.println(m2_maxVal);
}

void loop() {
  // Attempt to set the control settings for both motors
  // The function takes many parameters corresponding to the settings for each motor.
  bool success = controller.SetCtrlSettings(MOTOR_ADDRESS,
                                             m1_minDB, m1_maxDB, m1_minLimits, m1_maxLimits, m1_center, m1_minVal, m1_maxVal,
                                             m2_minDB, m2_maxDB, m2_minLimits, m2_maxLimits, m2_center, m2_minVal, m2_maxVal);

  if (success) {
    Serial.println("SETCTRLSETTINGS command successful.");
    // Optionally, read back the settings to verify (using GETCTRLSETTINGS)
    uint8_t read_m1_minDB, read_m1_maxDB, read_m2_minDB, read_m2_maxDB;
    uint16_t read_m1_minLimits, read_m1_maxLimits, read_m1_center, read_m1_minVal, read_m1_maxVal;
    uint16_t read_m2_minLimits, read_m2_maxLimits, read_m2_center, read_m2_minVal, read_m2_maxVal;

    if (controller.GetCtrlSettings(MOTOR_ADDRESS,
                                    read_m1_minDB, read_m1_maxDB, read_m1_minLimits, read_m1_maxLimits, read_m1_center, read_m1_minVal, read_m1_maxVal,
                                    read_m2_minDB, read_m2_maxDB, read_m2_minLimits, read_m2_maxLimits, read_m2_center, read_m2_minVal, read_m2_maxVal)) {
        Serial.println("Verified Control Settings:");
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
    } else {
        Serial.println("Failed to read Control Settings for verification.");
    }
  } else {
    Serial.println("SETCTRLSETTINGS command failed.");
    Serial.println("Check wiring, power, address, and baud rate.");
  }

  // Control settings are typically set once in setup or on configuration changes.
  // For this example, we'll add a long delay.
  delay(10000); // Wait 10 seconds before attempting again
}