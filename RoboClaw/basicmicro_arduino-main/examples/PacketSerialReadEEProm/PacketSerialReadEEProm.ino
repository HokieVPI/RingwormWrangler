/**
 * Basicmicro Library Example: READEEPROM (252)
 *
 * Demonstrates reading a 16-bit value from a specific address in the controller's
 * internal EEPROM. This is typically used for accessing small amounts of persistent
 * storage for custom parameters or flags not covered by other configuration commands.
 *
 * The EEPROM address is an 8-bit value (0-255). The value read is 16-bit.
 *
 * !!! IMPORTANT: The contents and organization of the EEPROM are controller-model-specific
 * and often undocumented publicly. Use caution when reading from or writing to EEPROM
 * addresses if you do not know their purpose. Writing to incorrect addresses can
 * corrupt controller configuration. This command is typically only available via
 * USB communications based on the library comment, but the library includes a
 * function for serial. Verify functionality with your specific controller and interface. !!!
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

// Define the EEPROM address to read from (0-255)
// !!! Consult your controller's documentation for meaningful addresses !!!
uint8_t eeprom_address_to_read = 0; // Example: Read from address 0


void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro READEEPROM Example");
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

  Serial.print("Attempting to read 16-bit value from EEPROM Address: "); Serial.println(eeprom_address_to_read);
  Serial.println("NOTE: EEPROM contents are controller-specific and potentially undocumented.");
}

void loop() {
  // Variable to store the read 16-bit value from EEPROM
  uint16_t read_value;

  // Attempt to read the 16-bit value from the specified EEPROM address
  // The function returns true on success, false on failure.
  // The value is stored in read_value.
  bool success = controller.ReadEEPROM(MOTOR_ADDRESS, eeprom_address_to_read, read_value);

  if (success) {
    Serial.println("READEEPROM command successful.");
    Serial.print("Value at EEPROM Address "); Serial.print(eeprom_address_to_read);
    Serial.print(": "); Serial.println(read_value); // Print as decimal
    Serial.print(" (0x"); Serial.print(read_value, HEX); Serial.println(")"); // Print as hex
  } else {
    Serial.println("READEEPROM command failed.");
    Serial.println("Check wiring, power, address, baud rate, and controller configuration.");
    Serial.println("This command may not be supported via serial or the address is invalid.");
  }

  // Wait a few seconds before reading again
  delay(3000); // Reading EEPROM periodically might be needed for some parameters
}