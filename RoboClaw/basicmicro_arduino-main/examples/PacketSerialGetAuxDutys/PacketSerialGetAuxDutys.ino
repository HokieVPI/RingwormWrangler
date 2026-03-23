/**
 * Basicmicro Library Example: GETAUXDUTYS (104)
 *
 * Demonstrates reading the current PWM duty cycle values configured for the
 * auxiliary output pins (S3, S4, S5, CTRL1, CTRL2). The availability and
 * function of these pins depend on the controller model and configuration
 * via SETPINFUNCTIONS (74).
 *
 * The returned duty cycle values are 16-bit unsigned integers (0 to 32767),
 * representing the PWM percentage (0% to 100%).
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

  Serial.println("Basicmicro GETAUXDUTYS Example");
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

  Serial.println("Attempting to read auxiliary pin duty cycles...");
}

void loop() {
  // Variables to store the read auxiliary duty cycles (0-32767)
  uint16_t s3_duty, s4_duty, s5_duty, ctrl1_duty, ctrl2_duty;

  // Attempt to read the auxiliary pin duty cycles
  // The function returns true on success, false on failure.
  // The values are stored in the provided variables.
  bool success = controller.GetAUXDutys(MOTOR_ADDRESS,
                                       s3_duty, s4_duty, s5_duty,
                                       ctrl1_duty, ctrl2_duty);

  if (success) {
    Serial.println("GETAUXDUTYS command successful.");
    Serial.println("Current Auxiliary Duty Cycles (0-32767):");
    Serial.print("  S3 Duty: "); Serial.println(s3_duty);
    Serial.print("  S4 Duty: "); Serial.println(s4_duty);
    Serial.print("  S5 Duty: "); Serial.println(s5_duty);
    Serial.print("  CTRL1 Duty: "); Serial.println(ctrl1_duty);
    Serial.print("  CTRL2 Duty: "); Serial.println(ctrl2_duty);
  } else {
    Serial.println("GETAUXDUTYS command failed.");
    Serial.println("Check wiring, power, address, and baud rate.");
    Serial.println("Ensure auxiliary outputs are configured on the controller.");
  }

  // Wait a few seconds before reading again
  delay(2000); // Reading settings infrequently is fine
}