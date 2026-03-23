/**
 * Basicmicro Library Example: SETMAINVOLTAGES (57)
 *
 * Demonstrates setting the minimum and maximum voltage limits for the main
 * battery (motor power supply). The controller can be configured to report
 * warnings or errors if the voltage falls outside these limits.
 *
 * The voltage values are specified in tenths of a volt (e.g., 120 = 12.0V).
 * An 'autoMax' flag can also be set.
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

// Define example main battery voltage limits in tenths of a volt
uint16_t main_batt_min_volts_tenths = 100; // 10.0V
uint16_t main_batt_max_volts_tenths = 200; // 20.0V (Example max, adjust for your battery)
uint8_t  main_batt_auto_max = 0;          // 0 = Manual max, 1 = Auto max (check controller docs)


void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro SETMAINVOLTAGES Example");
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

  Serial.print("Attempting to set Main Battery Min/Max: ");
  Serial.print((float)main_batt_min_volts_tenths / 10.0, 1); Serial.print("V / ");
  Serial.print((float)main_batt_max_volts_tenths / 10.0, 1); Serial.print("V, AutoMax: ");
  Serial.println(main_batt_auto_max);
}

void loop() {
  // Attempt to set the main battery voltage limits
  // The function returns true on success, false on failure.
  bool success = controller.SetMainVoltages(MOTOR_ADDRESS,
                                             main_batt_min_volts_tenths,
                                             main_batt_max_volts_tenths,
                                             main_batt_auto_max);

  if (success) {
    Serial.println("SETMAINVOLTAGES command successful.");
    // Optionally, read back the settings to verify
    uint16_t read_min, read_max;
    uint8_t read_autoMax;
    if (controller.ReadMinMaxMainVoltages(MOTOR_ADDRESS, read_min, read_max, read_autoMax)) {
        Serial.print("Verified Main Battery Min/Max: ");
        Serial.print((float)read_min / 10.0, 1); Serial.print("V / ");
        Serial.print((float)read_max / 10.0, 1); Serial.print("V, AutoMax: ");
        Serial.println(read_autoMax);
    } else {
        Serial.println("Failed to read Main Battery Voltages for verification.");
    }
  } else {
    Serial.println("SETMAINVOLTAGES command failed.");
    Serial.println("Check wiring, power, address, and baud rate.");
  }

  // Voltage limits are typically set once in setup or on configuration changes.
  // For this example, we'll add a long delay.
  delay(10000); // Wait 10 seconds before attempting again
}