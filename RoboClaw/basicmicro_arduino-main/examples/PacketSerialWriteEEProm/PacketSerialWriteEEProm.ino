/**
 * Basicmicro Library Example: WRITEEEPROM (253)
 *
 * Demonstrates writing a 16-bit value to a specific address in the controller's
 * internal EEPROM. This allows storing small amounts of persistent data.
 *
 * The EEPROM address is an 8-bit value (0-255). The value to write is 16-bit.
 *
 * !!! IMPORTANT: The contents and organization of the EEPROM are controller-model-specific
 * and often undocumented publicly. Use extreme caution when reading from or writing to EEPROM
 * addresses if you do not know their purpose. Writing to incorrect addresses can
 * permanently corrupt controller configuration and may render the device unusable.
 * This command is typically only available via USB communications based on the library comment,
 * but the library includes a function for serial. Verify functionality with your
 * specific controller and interface. !!!
 *
 * Note: Writing to EEPROM directly modifies the non-volatile memory. A separate
 * WRITENVM (94) command is usually for saving RAM settings, while this command
 * bypasses RAM and writes directly to EEPROM. This command itself does NOT typically
 * cause a controller reset, but interacting with EEPROM can be sensitive.
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

// Define the EEPROM address to write to (0-255) and the value to write (16-bit)
// !!! Consult your controller's documentation for safe addresses and values !!!
uint8_t eeprom_address_to_write = 0; // Example: Write to address 0
uint16_t value_to_write = 0xABCD;   // Example 16-bit value to write


void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro WRITEEEPROM Example");
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
  delay(100); // Short delay

  Serial.println("!!! WARNING: Attempting to write directly to EEPROM.");
  Serial.println("!!! This is potentially dangerous and can corrupt settings.");
  Serial.println("!!! Ensure you know the purpose of the EEPROM address.");
  Serial.print("Attempting to write value 0x"); Serial.print(value_to_write, HEX);
  Serial.print(" to EEPROM Address: "); Serial.println(eeprom_address_to_write);
}

void loop() {
  // Attempt to write the 16-bit value to the specified EEPROM address
  // The function returns true on success, false on failure.
  bool success = controller.WriteEEPROM(MOTOR_ADDRESS, eeprom_address_to_write, value_to_write);

  if (success) {
    Serial.println("WRITEEEPROM command successful. Value written to EEPROM.");
    Serial.println("Verification requires reading the EEPROM address.");
    // Optionally, read back the value to verify (using READEEPROM)
    /*
    uint16_t read_value;
    if (controller.ReadEEPROM(MOTOR_ADDRESS, eeprom_address_to_write, read_value)) {
        Serial.print("Verified value at EEPROM Address "); Serial.print(eeprom_address_to_write);
        Serial.print(": "); Serial.println(read_value);
        if (read_value == value_to_write) {
            Serial.println("  Value matches the written value.");
        } else {
            Serial.println("  Value DOES NOT match the written value.");
        }
    } else {
        Serial.println("Failed to read EEPROM for verification.");
    }
    */

  } else {
    Serial.println("WRITEEEPROM command failed.");
    Serial.println("Check wiring, power, address, baud rate, and controller configuration.");
    Serial.println("This command may not be supported via serial or the address/value is invalid.");
  }

  // Writing EEPROM is a configuration step, done once.
  // The loop will pause indefinitely after the first attempt.
  while(true); // Stop execution after setup and the single command attempt
}