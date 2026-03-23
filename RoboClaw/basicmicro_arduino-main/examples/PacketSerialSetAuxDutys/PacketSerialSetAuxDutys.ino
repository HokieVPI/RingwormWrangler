/**
 * Basicmicro Library Example: SETAUXDUTYS (102)
 *
 * Demonstrates setting the PWM duty cycles for the auxiliary output pins (S3, S4, S5, CTRL1, CTRL2).
 * The availability and function of these pins depend on the controller model and configuration
 * via SETPINFUNCTIONS (74).
 *
 * The duty cycle values are 16-bit unsigned integers (0 to 32767), representing
 * the PWM percentage (0% to 100%).
 *
 * Note: This command sets the duty cycles in the controller's RAM. To make them
 * permanent so they persist across power cycles, you must send a WRITENVM (94)
 * command afterwards. Be aware that WRITENVM will cause the controller to reset.
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

// Define example duty cycle values for the auxiliary pins (0-32767)
uint16_t s3_duty = 10000;   // Example: ~30.5% duty
uint16_t s4_duty = 20000;   // Example: ~61% duty
uint16_t s5_duty = 30000;   // Example: ~91.5% duty
uint16_t ctrl1_duty = 5000; // Example: ~15% duty
uint16_t ctrl2_duty = 15000; // Example: ~45.8% duty

void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("Basicmicro SETAUXDUTYS Example");
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

  Serial.println("Attempting to set auxiliary pin duty cycles...");
  Serial.print("S3: "); Serial.print(s3_duty);
  Serial.print(", S4: "); Serial.print(s4_duty);
  Serial.print(", S5: "); Serial.print(s5_duty);
  Serial.print(", CTRL1: "); Serial.print(ctrl1_duty);
  Serial.print(", CTRL2: "); Serial.println(ctrl2_duty);
}

void loop() {
  // Attempt to set the auxiliary pin duty cycles
  // The function returns true on success, false on failure.
  bool success = controller.SetAUXDutys(MOTOR_ADDRESS,
                                        s3_duty, s4_duty, s5_duty,
                                        ctrl1_duty, ctrl2_duty);

  if (success) {
    Serial.println("SETAUXDUTYS command successful. Duty cycles set in RAM.");
    Serial.println("Remember to use WRITENVM (94) to save permanently (causes reset).");
    // Optionally, read back the settings to verify (using GETAUXDUTYS)
    /*
    uint16_t read_s3, read_s4, read_s5, read_ctrl1, read_ctrl2;
    if (controller.GetAUXDutys(MOTOR_ADDRESS, read_s3, read_s4, read_s5, read_ctrl1, read_ctrl2)) {
        Serial.println("Verified Auxiliary Duty Cycles:");
        Serial.print("  S3: "); Serial.print(read_s3);
        Serial.print(", S4: "); Serial.print(read_s4);
        Serial.print(", S5: "); Serial.print(read_s5);
        Serial.print(", CTRL1: "); Serial.print(read_ctrl1);
        Serial.print(", CTRL2: "); Serial.println(read_ctrl2);
    } else {
        Serial.println("Failed to read Auxiliary Duty Cycles for verification.");
    }
    */

    // Example of saving to NVM afterwards (optional, and causes reset!)
    /*
    Serial.println("Attempting to save duty cycles to NVM (WRITENVM)...");
    if (controller.WriteNVM(MOTOR_ADDRESS)) {
        Serial.println("WRITENVM command successful. Duty cycles saved.");
        Serial.println("Controller is now resetting. Wait a few seconds before sending more commands.");
        delay(5000); // Wait for controller to reset and boot
    } else {
        Serial.println("WRITENVM command failed. Duty cycles NOT saved.");
    }
    */

  } else {
    Serial.println("SETAUXDUTYS command failed.");
    Serial.println("Check wiring, power, address, baud rate, and controller configuration.");
  }

  // Auxiliary duty cycles are typically set once in setup or infrequently.
  // For this example, we'll add a long delay.
  delay(10000); // Wait 10 seconds before attempting again
}