/**
 * Basicmicro Library Example: SETSTREAM (145) - MCP Only
 *
 * Demonstrates setting the communication parameters for a specific "Stream"
 * channel on MCP series controllers. Stream channels are typically used to
 * configure UART ports for specific protocols or purposes (e.g., Serial, Modbus).
 *
 * This command takes parameters for the stream index, type, baud rate, and timeout.
 *
 * !!! IMPORTANT: This command is marked as "MCP only". It may not work on
 * Sabertooth or Kangaroo controllers. The meaning of the parameters depends
 * heavily on the 'streamType' and the specific MCP model. Consult your
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

// Define example parameters for a Stream channel (e.g., configuring a serial port)
// !!! These values are examples. You MUST consult your controller's
// manual for the correct values for your desired stream type and behavior. !!!

uint8_t stream_index = 0;      // Example: Configure Stream 0 (check documentation for available indexes)
uint8_t stream_type = 0;       // Example: Type 0 (e.g., Basic Serial, check documentation)
uint32_t stream_baudrate = 9600; // Example: Baud rate 9600
uint32_t stream_timeout = 1000;  // Example: Timeout 1000 ms (32-bit value, units depend on controller)


void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro SETSTREAM Example (MCP Only)");
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

  Serial.print("Attempting to set Stream "); Serial.print(stream_index); Serial.println(" configuration...");
  Serial.println("NOTE: This command is for MCP controllers and parameters are complex/model-specific.");
  Serial.print("Type: "); Serial.print(stream_type);
  Serial.print(", Baudrate: "); Serial.print(stream_baudrate);
  Serial.print(", Timeout: "); Serial.println(stream_timeout);
}

void loop() {
  // Attempt to set the stream configuration
  // This command is typically only supported by MCP series controllers.
  // The function returns true on success, false on failure.
  bool success = controller.SetStream(MOTOR_ADDRESS,
                                      stream_index, stream_type,
                                      stream_baudrate, stream_timeout);

  if (success) {
    Serial.println("SETSTREAM command successful. Configuration set in RAM.");
    Serial.println("Remember to use WRITENVM (94) to save permanently (causes reset).");
    // Optionally, read back the setting to verify (using GETSTREAMS - note it reads ALL streams)
    /*
    uint8_t count;
    Basicmicro::StreamConfig configs[2]; // Assuming max 2 streams, adjust size as needed
    if (controller.GetStreams(MOTOR_ADDRESS, count, configs, sizeof(configs)/sizeof(configs[0]))) {
        Serial.println("Verified Stream Configurations:");
        // You would need to iterate through 'configs' array and print details for each stream
        // based on 'count'. This is verbose, so omitted here.
        Serial.print("Read "); Serial.print(count); Serial.println(" stream configurations.");
    } else {
        Serial.println("Failed to read Stream Configurations for verification.");
    }
    */

    // Example of saving to NVM afterwards (optional, and causes reset!)
    /*
    Serial.println("Attempting to save stream config to NVM (WRITENVM)...");
    if (controller.WriteNVM(MOTOR_ADDRESS)) {
        Serial.println("WRITENVM command successful. Config saved.");
        Serial.println("Controller is now resetting. Wait a few seconds before sending more commands.");
        delay(5000); // Wait for controller to reset and boot
    } else {
        Serial.println("WRITENVM command failed. Config NOT saved.");
    }
    */

  } else {
    Serial.println("SETSTREAM command failed.");
    Serial.println("Check wiring, power, address, baud rate, controller model, and parameter validity.");
    Serial.println("This command may not be supported by your controller or parameters are out of range/invalid type.");
  }

  // Stream settings are typically set once in setup or infrequently.
  // For this example, we'll add a long delay.
  delay(10000); // Wait 10 seconds before attempting again
}