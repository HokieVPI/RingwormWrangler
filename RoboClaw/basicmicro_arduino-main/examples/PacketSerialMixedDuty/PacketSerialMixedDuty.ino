/**
 * Basicmicro Library Example: MIXEDDUTY (34)
 *
 * Demonstrates setting the PWM duty cycles directly for both Motor 1 and Motor 2
 * in a single command. This bypasses PID speed or position control and applies
 * raw PWM values. The duty cycles are signed 16-bit values (-32768 to +32767).
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

// Define example duty cycle values for Motor 1 and Motor 2
// Signed 16-bit: -32768 (full reverse) to 32767 (full forward)
int16_t motor1_duty = 10000; // Example: ~30% forward
int16_t motor2_duty = -10000; // Example: ~30% reverse

void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro MIXEDDUTY Example");
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

  Serial.print("Attempting to set Motor 1 duty: "); Serial.print(motor1_duty);
  Serial.print(", Motor 2 duty: "); Serial.println(motor2_duty);
}

void loop() {
  // Attempt to set the PWM duty cycles for both motors
  // The function expects uint16_t, so we cast our signed int16_t values.
  // This results in the correct two's complement representation being sent.
  bool success = controller.DutyM1M2(MOTOR_ADDRESS, (uint16_t)motor1_duty, (uint16_t)motor2_duty);

  if (success) {
    Serial.println("MIXEDDUTY command successful.");
  } else {
    Serial.println("MIXEDDUTY command failed.");
    Serial.println("Check wiring, power, address, and baud rate.");
  }

  // Change the duty cycles periodically
  static unsigned long lastChange = 0;
  if (millis() - lastChange > 3000) { // Change every 3 seconds
      lastChange = millis();
      // Swap directions
      int16_t temp_duty = motor1_duty;
      motor1_duty = motor2_duty;
      motor2_duty = temp_duty;

      Serial.print("Switching Motor 1 duty: "); Serial.print(motor1_duty);
      Serial.print(", Motor 2 duty: "); Serial.println(motor2_duty);
  }

  // Short delay to prevent flooding serial
  delay(50);
}