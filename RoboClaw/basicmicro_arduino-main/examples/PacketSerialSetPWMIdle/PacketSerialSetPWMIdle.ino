/**
 * Basicmicro Library Example: SETPWMIDLE (160)
 *
 * Demonstrates setting the PWM idle behavior for both Motor 1 and Motor 2.
 * The controller can be configured to either coast or actively brake
 * the motor after a period of inactivity at zero command input (e.g., zero speed).
 * An idle delay can also be configured.
 *
 * The delay is specified in seconds (0.0 to 12.7s). The mode is either Coast (false)
 * or Brake (true).
 *
 * Note: This command sets the behavior in the controller's RAM. To make it
 * permanent so it persists across power cycles, you must send a
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

// Define example PWM Idle parameters
float m1_idle_delay_sec = 1.0; // Example: 1.0 seconds delay for M1
bool  m1_idle_mode_brake = true;  // Example: M1 brakes after delay

float m2_idle_delay_sec = 0.5; // Example: 0.5 seconds delay for M2
bool  m2_idle_mode_brake = false; // Example: M2 coasts after delay


void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro SETPWMIDLE Example");
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

  Serial.print("Attempting to set M1 Idle: Delay "); Serial.print(m1_idle_delay_sec, 1); Serial.print("s, Mode "); Serial.print(m1_idle_mode_brake ? "Brake" : "Coast");
  Serial.print(" | M2 Idle: Delay "); Serial.print(m2_idle_delay_sec, 1); Serial.print("s, Mode "); Serial.println(m2_idle_mode_brake ? "Brake" : "Coast");
}

void loop() {
  // Attempt to set the PWM idle parameters for both motors
  // The function returns true on success, false on failure.
  bool success = controller.SetPWMIdle(MOTOR_ADDRESS,
                                        m1_idle_delay_sec, m1_idle_mode_brake,
                                        m2_idle_delay_sec, m2_idle_mode_brake);

  if (success) {
    Serial.println("SETPWMIDLE command successful. Settings set in RAM.");
    Serial.println("Remember to use WRITENVM (94) to save permanently (causes reset).");
    // Optionally, read back the setting to verify (using GETPWMIDLE)
    /*
    float read_delay1, read_delay2;
    bool read_mode1, read_mode2;
    if (controller.GetPWMIdle(MOTOR_ADDRESS, read_delay1, read_mode1, read_delay2, read_mode2)) {
        Serial.println("Verified PWM Idle Settings:");
        Serial.print("  M1 Idle: Delay "); Serial.print(read_delay1, 1); Serial.print("s, Mode "); Serial.println(read_mode1 ? "Brake" : "Coast");
        Serial.print("  M2 Idle: Delay "); Serial.print(read_delay2, 1); Serial.print("s, Mode "); Serial.println(read_mode2 ? "Brake" : "Coast");
    } else {
        Serial.println("Failed to read PWM Idle Settings for verification.");
    }
    */

    // Example of saving to NVM afterwards (optional, and causes reset!)
    /*
    Serial.println("Attempting to save PWM Idle settings to NVM (WRITENVM)...");
    if (controller.WriteNVM(MOTOR_ADDRESS)) {
        Serial.println("WRITENVM command successful. Settings saved.");
        Serial.println("Controller is now resetting. Wait a few seconds before sending more commands.");
        delay(5000); // Wait for controller to reset and boot
    } else {
        Serial.println("WRITENVM command failed. Settings NOT saved.");
    }
    */

  } else {
    Serial.println("SETPWMIDLE command failed.");
    Serial.println("Check wiring, power, address, baud rate, and controller configuration.");
    Serial.println("Ensure valid delay values are used (0.0 to 12.7).");
  }

  // PWM idle settings are typically set once in setup or on configuration changes.
  // For this example, we'll add a long delay.
  delay(10000); // Wait 10 seconds before attempting again
}