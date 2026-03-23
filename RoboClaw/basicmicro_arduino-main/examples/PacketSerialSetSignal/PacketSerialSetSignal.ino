/**
 * Basicmicro Library Example: SETSIGNAL (143) - MCP Only
 *
 * Demonstrates setting the configuration parameters for a specific "Signal"
 * input channel on MCP series controllers. Signal channels are used to map
 * physical inputs (like analog voltages, RC PWM pulses, encoders, etc.)
 * to internal controller functions or motor commands.
 *
 * This command takes a large number of parameters to define the mapping,
 * scaling, filtering, deadband, limits, etc., for the signal.
 *
 * !!! IMPORTANT: This command is marked as "MCP only". It may not work on
 * Sabertooth or Kangaroo controllers. The meaning of the parameters depends
 * heavily on the 'signalType' and the specific MCP model. Consult your
 * controller's documentation for detailed parameter definitions and valid values. !!!
 *
 * Note: This command sets the configuration in the controller's RAM. To make it
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

// Define example parameters for a Signal channel (e.g., configuring an Analog input)
// !!! These values are examples. You MUST consult your controller's
// manual for the correct values for your desired signal type and behavior. !!!

uint8_t signal_index = 0;       // Example: Configure Signal 0 (check documentation for available indexes)
uint8_t signal_type = 1;        // Example: Type 1 (e.g., Analog Input, check documentation)
uint8_t signal_mode = 0;        // Example: Mode 0 (e.g., Unipolar, check documentation)
uint8_t signal_target = 0;      // Example: Target 0 (e.g., M1 Speed, check documentation)
uint16_t signal_minAction = 0;   // Example: Minimum output action (16-bit, e.g., corresponds to min speed)
uint16_t signal_maxAction = 32767; // Example: Maximum output action (16-bit, e.g., corresponds to max speed)
uint8_t signal_lowpass = 1;     // Example: Lowpass filter setting (8-bit)
uint32_t signal_timeout = 1000;  // Example: Timeout (32-bit, e.g., milliseconds)
int32_t signal_loadhome = 0;    // Example: Load Home position (32-bit signed)
int32_t signal_minVal = 0;      // Example: Minimum raw input value (32-bit signed, e.g., ADC counts)
int32_t signal_maxVal = 4095;   // Example: Maximum raw input value (32-bit signed, e.g., ADC counts for 12-bit ADC)
int32_t signal_center = 2048;   // Example: Center raw input value (32-bit signed)
uint32_t signal_deadband = 50;   // Example: Deadband around center (32-bit unsigned)
uint32_t signal_powerexp = 1000; // Example: Power exponent (32-bit unsigned, e.g., 1000 for linear)
uint32_t signal_minout = 0;      // Example: Minimum scaled output (32-bit unsigned)
uint32_t signal_maxout = 32767;  // Example: Maximum scaled output (32-bit unsigned)
uint32_t signal_powermin = 0;    // Example: Power minimum (32-bit unsigned)
uint32_t signal_potentiometer = 0; // Example: Potentiometer type/setting (32-bit unsigned)


void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro SETSIGNAL Example (MCP Only)");
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

  Serial.print("Attempting to set Signal "); Serial.print(signal_index); Serial.println(" configuration...");
  Serial.println("NOTE: This command is for MCP controllers and parameters are complex/model-specific.");
}

void loop() {
  // Attempt to set the signal configuration
  // This command is typically only supported by MCP series controllers.
  // The function returns true on success, false on failure.
  bool success = controller.SetSignal(MOTOR_ADDRESS,
                                       signal_index, signal_type, signal_mode,
                                       signal_target, signal_minAction, signal_maxAction, signal_lowpass,
                                       signal_timeout, signal_loadhome, signal_minVal, signal_maxVal,
                                       signal_center, signal_deadband, signal_powerexp, signal_minout,
                                       signal_maxout, signal_powermin, signal_potentiometer);

  if (success) {
    Serial.println("SETSIGNAL command successful. Configuration set in RAM.");
    Serial.println("Remember to use WRITENVM (94) to save permanently (causes reset).");
    // Optionally, read back the setting to verify (using GETSIGNALS - note it reads ALL signals)
    /*
    uint8_t count;
    Basicmicro::SignalConfig configs[5]; // Assuming max 5 signals, adjust size as needed
    if (controller.GetSignals(MOTOR_ADDRESS, count, configs, sizeof(configs)/sizeof(configs[0]))) {
        Serial.println("Verified Signal Configurations:");
        // You would need to iterate through 'configs' array and print details for each signal
        // based on 'count'. This is verbose, so omitted here.
         Serial.print("Read "); Serial.print(count); Serial.println(" signal configurations.");
    } else {
        Serial.println("Failed to read Signal Configurations for verification.");
    }
    */

    // Example of saving to NVM afterwards (optional, and causes reset!)
    /*
    Serial.println("Attempting to save signal config to NVM (WRITENVM)...");
    if (controller.WriteNVM(MOTOR_ADDRESS)) {
        Serial.println("WRITENVM command successful. Config saved.");
        Serial.println("Controller is now resetting. Wait a few seconds before sending more commands.");
        delay(5000); // Wait for controller to reset and boot
    } else {
        Serial.println("WRITENVM command failed. Config NOT saved.");
    }
    */

  } else {
    Serial.println("SETSIGNAL command failed.");
    Serial.println("Check wiring, power, address, baud rate, controller model, and parameter validity.");
    Serial.println("This command may not be supported by your controller or parameters are out of range/invalid type.");
  }

  // Signal settings are typically set once in setup or infrequently.
  // For this example, we'll add a long delay.
  delay(10000); // Wait 10 seconds before attempting again
}