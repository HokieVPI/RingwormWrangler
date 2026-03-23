/**
 * Basicmicro Library Example: GETSIGNALSDATA (147) - MCP Only
 *
 * Demonstrates reading the current processed data values for all "Signal"
 * input channels on MCP series controllers. This provides the controller's
 * interpretation of the physical input signals after applying scaling,
 * filtering, deadband, etc., as configured by SETSIGNAL (143).
 *
 * The returned data includes command, position, percent, speed, and speeds
 * (all 32-bit values) for each signal channel.
 *
 * !!! IMPORTANT: This command is marked as "MCP only". It may not work on
 * Sabertooth or Kangaroo controllers. The meaning of the returned parameters
 * depends heavily on the 'signalType' and 'target' configured for each signal
 * via SETSIGNAL. Consult your controller's documentation for detailed parameter definitions. !!!
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

// Buffer to store the signal data.
// Choose a size large enough for the maximum number of signals your controller has.
// The library header doesn't specify a max, but 5-10 is a common number of signals.
#define MAX_SIGNALS 5
Basicmicro::SignalData signalData[MAX_SIGNALS];


void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro GETSIGNALSDATA Example (MCP Only)");
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

  Serial.println("Attempting to read signal data...");
  Serial.println("NOTE: This command is for MCP controllers.");

  // To see meaningful data, you would first need to configure Signal inputs using SETSIGNAL (143).
  // E.g., configure a pin as an analog input for motor speed control.
}

void loop() {
  // Variable to store the number of signals reported by the controller
  uint8_t signal_count = 0; // Initialize to 0

  // Attempt to read the signal data for all configured signals
  // This command is typically only supported by MCP series controllers.
  // The function returns true on success, false on failure.
  // The number of signals is stored in signal_count, and the data
  // is stored in the signalData array (up to MAX_SIGNALS).
  bool success = controller.GetSignalsData(MOTOR_ADDRESS, signal_count, signalData, MAX_SIGNALS);

  if (success) {
    Serial.println("GETSIGNALSDATA command successful.");
    Serial.print("Reported Signal Data Count: "); Serial.println(signal_count);
    Serial.println("Current Signal Data:");

    // Iterate up to the reported count (or MAX_SIGNALS if reported count is larger)
    uint8_t display_count = min(signal_count, MAX_SIGNALS);
    for (uint8_t i = 0; i < display_count; i++) {
        Serial.print("  Signal Index "); Serial.print(i); Serial.println(":");
        Serial.print("    command: "); Serial.println(signalData[i].command);
        Serial.print("    position: "); Serial.println(signalData[i].position); // May be signed based on config?
        Serial.print("    percent: "); Serial.println(signalData[i].percent); // May be signed based on config?
        Serial.print("    speed: "); Serial.println(signalData[i].speed);     // May be signed based on config?
        Serial.print("    speeds: "); Serial.println(signalData[i].speeds);   // May be signed based on config?
    }
     if (signal_count > MAX_SIGNALS) {
        Serial.print("(Only displaying first "); Serial.print(MAX_SIGNALS); Serial.println(" data entries due to buffer size)");
    }
    Serial.println("NOTE: Interpretation of these values depends on the signal configuration (SETSIGNAL).");


  } else {
    Serial.println("GETSIGNALSDATA command failed.");
    Serial.println("Check wiring, power, address, baud rate, and controller model.");
    Serial.println("This command may not be supported by your controller.");
  }

  // Wait a short time before reading again
  delay(100); // Reading data from signals periodically is common
}