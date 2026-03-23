/**
 * Basicmicro Library Example: SETSPEEDERRORLIMIT (109)
 *
 * Demonstrates setting the maximum allowable speed error limit for Motor 1 and Motor 2.
 * If the difference between the target speed and the actual measured speed for a motor
 * exceeds its configured limit while in Velocity PID mode, an error flag (ERROR_SPEED1 or ERROR_SPEED2)
 * will be set in the controller's status register.
 *
 * The limit is specified as an unsigned 16-bit integer, representing the maximum
 * allowable error in encoder counts per second.
 *
 * Note: This command sets the limits in the controller's RAM. To make them
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

// Define example speed error limits (unsigned 16-bit) in counts/sec
uint16_t m1_speed_error_limit = 50;  // Motor 1 error limit (e.g., 50 counts/sec)
uint16_t m2_speed_error_limit = 60;  // Motor 2 error limit (e.g., 60 counts/sec)


void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro SETSPEEDERRORLIMIT Example");
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

  Serial.print("Attempting to set Motor 1 Speed Error Limit to: "); Serial.print(m1_speed_error_limit);
  Serial.print(", Motor 2 Speed Error Limit to: "); Serial.println(m2_speed_error_limit);
  Serial.println(" (counts/sec)");
}

void loop() {
  // Attempt to set the speed error limits for both motors
  // The function expects uint16_t values.
  bool success = controller.SetSpeedErrorLimit(MOTOR_ADDRESS, m1_speed_error_limit, m2_speed_error_limit);

  if (success) {
    Serial.println("SETSPEEDERRORLIMIT command successful. Limits set in RAM.");
    Serial.println("Remember to use WRITENVM (94) to save permanently (causes reset).");
    // Optionally, read back the setting to verify (using GETSPEEDERRORLIMIT)
    /*
    uint16_t read_limit1, read_limit2;
    if (controller.GetSpeedErrorLimit(MOTOR_ADDRESS, read_limit1, read_limit2)) {
        Serial.println("Verified Speed Error Limits:");
        Serial.print("  Motor 1 Limit: "); Serial.println(read_limit1);
        Serial.print("  Motor 2 Limit: "); Serial.println(read_limit2);
    } else {
        Serial.println("Failed to read Speed Error Limits for verification.");
    }
    */

    // Example of saving to NVM afterwards (optional, and causes reset!)
    /*
    Serial.println("Attempting to save limits to NVM (WRITENVM)...");
    if (controller.WriteNVM(MOTOR_ADDRESS)) {
        Serial.println("WRITENVM command successful. Limits saved.");
        Serial.println("Controller is now resetting. Wait a few seconds before sending more commands.");
        delay(5000); // Wait for controller to reset and boot
    } else {
        Serial.println("WRITENVM command failed. Limits NOT saved.");
    }
    */

  } else {
    Serial.println("SETSPEEDERRORLIMIT command failed.");
    Serial.println("Check wiring, power, address, baud rate, and controller configuration.");
  }

  // Speed error limits are typically set once in setup or on configuration changes.
  // For this example, we'll add a long delay.
  delay(10000); // Wait 10 seconds before attempting again
}