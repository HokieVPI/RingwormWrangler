/**
 * Basicmicro Library Example: SETPWMMODE (148)
 *
 * Demonstrates setting the PWM mode for Motor 1 and Motor 2.
 *
 * Valid modes typically include:
 * 0 = Inductive Mode
 * 1 = Resistive Mode
 *
 * The optimal mode depends on your load type (Brushed DC) and desired performance.
 * Resistive Mode must be used if a resistive load is being driven.
 * Resistive Mode also MAY be required on particularly LOW inductance Motors
 *
 * Note: This command sets the mode in the controller's RAM. To make it
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

// Define the desired PWM mode for Motor 1 and Motor 2 (0 or 1)
uint8_t motor1_pwm_mode = 1; // Example: Set M1 to Resistive Mode
uint8_t motor2_pwm_mode = 1; // Example: Set M2 to Resistive Mode

void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro SETPWMMODE Example");
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

  Serial.print("Attempting to set Motor 1 PWM Mode to: "); Serial.print(motor1_pwm_mode);
  Serial.print(", Motor 2 PWM Mode to: "); Serial.println(motor2_pwm_mode);
  Serial.println("(0=Inductive Mode, 1=Resistive Mode)");
}

void loop() {
  // Attempt to set the PWM mode for both motors
  // The function expects uint8_t values for both modes.
  bool success = controller.SetPWMMode(MOTOR_ADDRESS, motor1_pwm_mode, motor2_pwm_mode);

  if (success) {
    Serial.println("SETPWMMODE command successful. Modes set in RAM.");
    Serial.println("Remember to use WRITENVM (94) to save permanently (causes reset).");
    // Optionally, read back the setting to verify (using GETPWMMODE)
    /*
    uint8_t read_mode1, read_mode2;
    if (controller.GetPWMMode(MOTOR_ADDRESS, read_mode1, read_mode2)) {
        Serial.println("Verified PWM Modes:");
        Serial.print("  Motor 1 Mode: "); Serial.print(read_mode1);
        Serial.print(", Motor 2 Mode: "); Serial.println(read_mode2);
    } else {
        Serial.println("Failed to read PWM Modes for verification.");
    }
    */

    // Example of saving to NVM afterwards (optional, and causes reset!)
    /*
    Serial.println("Attempting to save PWM modes to NVM (WRITENVM)...");
    if (controller.WriteNVM(MOTOR_ADDRESS)) {
        Serial.println("WRITENVM command successful. Modes saved.");
        Serial.println("Controller is now resetting. Wait a few seconds before sending more commands.");
        delay(5000); // Wait for controller to reset and boot
    } else {
        Serial.println("WRITENVM command failed. Modes NOT saved.");
    }
    */

  } else {
    Serial.println("SETPWMMODE command failed.");
    Serial.println("Check wiring, power, address, baud rate, and controller configuration.");
    Serial.println("Ensure valid mode values are used (0 or 1).");
  }

  // PWM mode is typically set once in setup or on configuration changes.
  // For this example, we'll add a long delay.
  delay(10000); // Wait 10 seconds before attempting again
}