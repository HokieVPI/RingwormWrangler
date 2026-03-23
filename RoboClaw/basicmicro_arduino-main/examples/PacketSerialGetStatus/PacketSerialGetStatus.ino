/**
 * Basicmicro Library Example: GETSTATUS (73)
 *
 * Demonstrates reading a comprehensive set of status information from the
 * Basicmicro motor controller in a single command. This includes tick count,
 * state flags, temperatures, voltages, PWM values, currents, encoder counts,
 * speed setpoints, actual speeds, and error values.
 *
 * This example is useful for monitoring the overall health and operating
 * condition of the controller.
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

  Serial.println("Basicmicro GETSTATUS Example");
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

  Serial.println("Attempting to read comprehensive status...");
}

void loop() {
  // Declare variables to store all the status information
  uint32_t tick;
  uint32_t state;
  uint16_t temp1_tenths, temp2_tenths;
  uint16_t mainBattVoltage_tenths, logicBattVoltage_tenths;
  int16_t pwm1, pwm2;
  int16_t current1_mA, current2_mA;
  uint32_t enc1_u, enc2_u; // Read as unsigned, cast to signed for print
  uint32_t speed1_u, speed2_u; // Read as unsigned, cast to signed for print
  uint32_t ispeed1_u, ispeed2_u; // Read as unsigned, cast to signed for print
  uint16_t speedError1, speedError2;
  uint16_t posError1, posError2;

  // Attempt to retrieve the comprehensive status
  // The function returns true on success, false on failure.
  // All provided variables will be filled with the status data on success.
  bool success = controller.GetStatus(MOTOR_ADDRESS,
                                     tick, state,
                                     temp1_tenths, temp2_tenths,
                                     mainBattVoltage_tenths, logicBattVoltage_tenths,
                                     pwm1, pwm2, current1_mA, current2_mA,
                                     enc1_u, enc2_u, speed1_u, speed2_u,
                                     ispeed1_u, ispeed2_u, speedError1, speedError2,
                                     posError1, posError2);

  if (success) {
    Serial.println("GETSTATUS command successful.");
    Serial.println("--- Controller Status ---");
    Serial.print("Tick: "); Serial.println(tick);
    Serial.print("State Flags: 0x"); Serial.println(state, HEX);
    // You can decode specific state bits here if needed (e.g., using ERROR_ESTOP, WARN_MBATLOW etc.)
    if (state & controller.ERROR_ESTOP) Serial.println("  ERROR: E-Stop Active");
    if (state & controller.WARN_MBATLOW) Serial.println("  WARN: Main Battery Low");
    // Add more checks for other status bits as defined in Basicmicro.h

    Serial.print("Temp 1: "); Serial.print((float)temp1_tenths / 10.0, 1); Serial.println(" C");
    Serial.print("Temp 2: "); Serial.print((float)temp2_tenths / 10.0, 1); Serial.println(" C"); // May be 0 or incorrect on controllers without Temp2
    Serial.print("Main Batt: "); Serial.print((float)mainBattVoltage_tenths / 10.0, 1); Serial.println(" V");
    Serial.print("Logic Batt: "); Serial.print((float)logicBattVoltage_tenths / 10.0, 1); Serial.println(" V");

    Serial.print("PWM 1: "); Serial.print(pwm1); Serial.print(", PWM 2: "); Serial.println(pwm2); // Signed 16-bit
    Serial.print("Current 1: "); Serial.print(current1_mA); Serial.print(" mA, Current 2: "); Serial.print(current2_mA); Serial.println(" mA"); // Signed 16-bit

    Serial.print("Encoder 1: "); Serial.print((int32_t)enc1_u); Serial.print(", Encoder 2: "); Serial.println((int32_t)enc2_u); // Signed 32-bit counts
    Serial.print("Speed 1: "); Serial.print((int32_t)speed1_u); Serial.print(", Speed 2: "); Serial.println((int32_t)speed2_u); // Signed 32-bit counts/sec
    Serial.print("Instant Speed 1: "); Serial.print((int32_t)ispeed1_u); Serial.print(", Instant Speed 2: "); Serial.println((int32_t)ispeed2_u); // Signed 32-bit counts/sec

    Serial.print("Speed Error 1: "); Serial.print(speedError1); Serial.print(", Speed Error 2: "); Serial.println(speedError2); // Unsigned 16-bit
    Serial.print("Position Error 1: "); Serial.print(posError1); Serial.print(", Position Error 2: "); Serial.println(posError2); // Unsigned 16-bit

    Serial.println("-------------------------");

  } else {
    Serial.println("GETSTATUS command failed.");
    Serial.println("Check wiring, power, address, and baud rate.");
  }

  // Wait a short time before reading again
  delay(500); // Reading status frequently is common, but avoid flooding serial output
}