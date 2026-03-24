/**
 * Basicmicro Library Example: MIXEDSPEED2ACCEL (50)
 *
 * Demonstrates setting the target speeds for both Motor 1 and Motor 2
 * with independent acceleration rates in a single command.
 *
 * This command requires Velocity PID to be enabled and tuned for both motors.
 *
 * This example uses HardwareSerial (Serial1) for communication with the controller
 * and HardwareSerial (Serial) for debugging output. Adjust serial ports and
 * controller address as needed for your setup.
 */

 #include <Arduino.h>
 #include <Basicmicro.h>
 
 // Define the serial port for communication with the motor controller
 // On boards like Mega, Due, Leonardo, etc., use Serial1, Serial2, Serial3.
 #define CONTROLLER_SERIAL   Serial1
 
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
Basicmicro controller(&CONTROLLER_SERIAL, LIBRARY_READ_TIMEOUT);// fix
 
 // Define example acceleration and target speeds for both motors independently
 uint32_t motor1_accel = 25000;     // M1 Acceleration rate in counts/sec/sec
 int32_t motor1_target_speed = 10000; // M1 Target speed in counts/sec (signed)
 
 uint32_t motor2_accel = 25000;     // M2 Acceleration rate in counts/sec/sec
 int32_t motor2_target_speed = 10000; // M2 Target speed in counts/sec (signed)
 
 
 void setup() {
   // Initialize debug serial port
   Serial.begin(115200);
   while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)
 
   Serial.println("Basicmicro MIXEDSPEED2ACCEL Example");
   Serial.print("Connecting to controller on ");
   // Print the name of the serial port being used (if possible)
   #if defined(CONTROLLER_SERIAL) && !defined(RX_PIN) // Check if using HardwareSerial and not SoftwareSerial
     Serial.println("Hardware Serial");
   #elif defined(RX_PIN) // Check if SoftwareSerial pins are defined
     Serial.println("Software Serial");
   #else
     Serial.println("Unknown Serial type");
   #endif
 
 
   // Initialize the communication serial port for the controller
   controller.begin(38400); // Ensure baud rate matches your controller
   delay(100); // Short delay to let the controller initialize after power-up or serial init
 
   // Note: For Speed commands to work, Velocity PID must be enabled and tuned for both motors.
   // You might need to send SETM1PID and SETM2PID here or ensure it's configured in the controller's NVM.
   // Example PID values (these are just placeholders, you need values tuned for your system):
   controller.SetM1VelocityPID(MOTOR_ADDRESS, 1.79279, 0.27940, 0.00000, 70620);
   controller.SetM2VelocityPID(MOTOR_ADDRESS, 1.74675, 0.26201, 0.00000, 69630);
 
   Serial.print("Attempting to set M1 Accel/Speed: "); Serial.print(motor1_accel); Serial.print("/"); Serial.print(motor1_target_speed);
   Serial.print(", M2 Accel/Speed: "); Serial.print(motor2_accel); Serial.print("/"); Serial.print(motor2_target_speed);
   Serial.println(" counts/sec");
 }
 
// Phase 0: Both motors  (10s)
// Phase 1: M1 only      (4s)
// Phase 2: Both motors  (10s)
// Phase 3: M2 only      (4s)
static uint8_t phase = 0;
static unsigned long phaseStart = 0;
static const unsigned long phaseDurations[] = {10000, 8500, 10000, 8500};

void loop() {
  unsigned long now = millis();

  if (phaseStart == 0) phaseStart = now;

  if (now - phaseStart >= phaseDurations[phase]) {
    phase = (phase + 1) % 4;
    phaseStart = now;

    const char *labels[] = {"Both motors", "M1 only", "Both motors", "M2 only"};
    Serial.print("Phase -> "); Serial.println(labels[phase]);
  }

  uint32_t m1_spd, m2_spd;

  switch (phase) {
    case 0: // Both motors
    case 2:
      m1_spd = (uint32_t)motor1_target_speed;
      m2_spd = (uint32_t)motor2_target_speed;
      break;
    case 1: // M1 only
      m1_spd = (uint32_t)motor1_target_speed;
      m2_spd = 0;
      break;
    case 3: // M2 only
      m1_spd = 0;
      m2_spd = (uint32_t)motor2_target_speed;
      break;
  }

  bool success = controller.SpeedAccelM1M2_2(MOTOR_ADDRESS,
                                              motor1_accel, m1_spd,
                                              motor2_accel, m2_spd);

  if (!success) {
    Serial.println("SpeedAccelM1M2_2 command failed.");
  }

  delay(50);
}