/**
 * Basicmicro Library Example: GETTEMP2 (83)
 *
 * Demonstrates reading the temperature from temperature sensor 2 on the
 * Basicmicro motor controller.
 *
 * !!! IMPORTANT: This command and sensor are ONLY valid on certain controller models.
 * On models without a second temperature sensor, this command may fail or return
 * an incorrect/default value. Consult your controller's documentation. !!!
 *
 * The returned value is in tenths of a degree Celsius (e.g., 250 = 25.0°C).
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

  Serial.println("Basicmicro GETTEMP2 Example");
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

  Serial.println("Attempting to read temperature sensor 2...");
  Serial.println("NOTE: This sensor may not be available on all controller models.");
}

void loop() {
  // Variable to store the read temperature value (in tenths of a degree Celsius)
  uint16_t temperature2_tenths;

  // Attempt to read the temperature from sensor 2
  // The function returns true on success, false on failure.
  // The value is stored in temperature2_tenths.
  bool success = controller.ReadTemp2(MOTOR_ADDRESS, temperature2_tenths);

  if (success) {
    Serial.println("GETTEMP2 command successful.");
    Serial.print("Temperature Sensor 2: ");
    Serial.print((float)temperature2_tenths / 10.0, 1); // Convert to float for printing
    Serial.println(" C");
  } else {
    Serial.println("GETTEMP2 command failed.");
    Serial.println("Check wiring, power, address, baud rate, or if your controller model has a second temperature sensor.");
  }

  // Wait a few seconds before reading again
  delay(1000); // Reading temperature periodically is common
}