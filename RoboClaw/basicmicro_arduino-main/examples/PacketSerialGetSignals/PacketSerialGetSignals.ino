/**
 * Basicmicro Library Example: GETSIGNALS (144) - MCP Only
 *
 * Demonstrates reading the configuration parameters for all "Signal" input
 * channels on MCP series controllers. This command returns the total number
 * of configured signals and their detailed settings.
 *
 * !!! IMPORTANT: This command is marked as "MCP only". It may not work on
 * Sabertooth or Kangaroo controllers. The meaning of the returned parameters
 * depends heavily on the 'signalType' and the specific MCP model. Consult your
 * controller's documentation for detailed parameter definitions. !!!
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

// Buffer to store the signal configurations.
// Choose a size large enough for the maximum number of signals your controller has.
// The library header doesn't specify a max, but 5-10 is a common number of signals.
#define MAX_SIGNALS 5
Basicmicro::SignalConfig signalConfigurations[MAX_SIGNALS];


void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro GETSIGNALS Example (MCP Only)");
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

  Serial.println("Attempting to read signal configurations...");
  Serial.println("NOTE: This command is for MCP controllers.");
}

void loop() {
  // Variable to store the number of signals reported by the controller
  uint8_t signal_count = 0; // Initialize to 0

  // Attempt to read the signal configurations
  // This command is typically only supported by MCP series controllers.
  // The function returns true on success, false on failure.
  // The number of signals is stored in signal_count, and the configurations
  // are stored in the signalConfigurations array (up to MAX_SIGNALS).
  bool success = controller.GetSignals(MOTOR_ADDRESS, signal_count, signalConfigurations, MAX_SIGNALS);

  if (success) {
    Serial.println("GETSIGNALS command successful.");
    Serial.print("Reported Signal Count: "); Serial.println(signal_count);
    Serial.println("Current Signal Configurations:");

    // Iterate up to the reported count (or MAX_SIGNALS if reported count is larger)
    uint8_t display_count = min(signal_count, MAX_SIGNALS);
    for (uint8_t i = 0; i < display_count; i++) {
        Serial.print("  Signal Index "); Serial.print(i); Serial.println(":");
        Serial.print("    type: "); Serial.println(signalConfigurations[i].type);
        Serial.print("    mode: "); Serial.println(signalConfigurations[i].mode);
        Serial.print("    target: "); Serial.println(signalConfigurations[i].target);
        Serial.print("    minAction: "); Serial.println(signalConfigurations[i].minAction);
        Serial.print("    maxAction: "); Serial.println(signalConfigurations[i].maxAction);
        Serial.print("    lowpass: "); Serial.println(signalConfigurations[i].lowpass);
        Serial.print("    timeout: "); Serial.println(signalConfigurations[i].timeout);
        Serial.print("    loadhome: "); Serial.println(signalConfigurations[i].loadhome);
        Serial.print("    minVal: "); Serial.println(signalConfigurations[i].minVal);
        Serial.print("    maxVal: "); Serial.println(signalConfigurations[i].maxVal);
        Serial.print("    center: "); Serial.println(signalConfigurations[i].center);
        Serial.print("    deadband: "); Serial.println(signalConfigurations[i].deadband);
        Serial.print("    powerexp: "); Serial.println(signalConfigurations[i].powerexp);
        Serial.print("    minout: "); Serial.println(signalConfigurations[i].minout);
        Serial.print("    maxout: "); Serial.println(signalConfigurations[i].maxout);
        Serial.print("    powermin: "); Serial.println(signalConfigurations[i].powermin);
        Serial.print("    potentiometer: "); Serial.println(signalConfigurations[i].potentiometer);
    }
     if (signal_count > MAX_SIGNALS) {
        Serial.print("(Only displaying first "); Serial.print(MAX_SIGNALS); Serial.println(" configurations due to buffer size)");
    }
    Serial.println("NOTE: Interpretation of these values is specific to your controller model.");


  } else {
    Serial.println("GETSIGNALS command failed.");
    Serial.println("Check wiring, power, address, baud rate, and controller model.");
    Serial.println("This command may not be supported by your controller.");
  }

  // Wait a few seconds before reading again
  delay(5000); // Reading settings infrequently is fine
}