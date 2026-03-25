/**
 * Basicmicro Library Example: MIXEDSPEED2ACCEL (50)
 *
 * Demonstrates setting the target speeds for both Motor 1 and Motor 2
 * with independent acceleration rates in a single command.
 *
 * This command requires Velocity PID to be enabled and tuned for both motors.
 *
 * Configured for Arduino Portenta C33 using hardware UART on pins 13 (RX) and 14 (TX).
 * USB Serial is used for debug output.
 */

#include <Arduino.h>
#include <Basicmicro.h>

// Hardware UART on Portenta C33: RX = pin 13, TX = pin 14
UART controllerSerial(14, 13);

#define MOTOR_ADDRESS        128
#define LIBRARY_READ_TIMEOUT 10000

Basicmicro controller(&controllerSerial, LIBRARY_READ_TIMEOUT);
 
 // Define example acceleration and target speeds for both motors independently
 uint32_t motor1_accel = 25000;     // M1 Acceleration rate in counts/sec/sec
 int32_t motor1_target_speed = 7062; // M1 Target speed in counts/sec (signed)
 
 uint32_t motor2_accel = 25000;     // M2 Acceleration rate in counts/sec/sec
 int32_t motor2_target_speed = 7062; // M2 Target speed in counts/sec (signed)
 
 
 void setup() {
   // Initialize debug serial port
   Serial.begin(115200);
   while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)
 
  Serial.println("Basicmicro MIXEDSPEED2ACCEL Example");
  Serial.println("Connecting to controller on Hardware Serial (pins 13/14)");
 
 
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
 
 void loop() {
   // Attempt to set the target speeds for both motors with independent acceleration
   // The speed parameters expect uint32_t, so we cast our signed int32_t values.
   bool success = controller.SpeedAccelM1M2_2(MOTOR_ADDRESS,
                                               motor1_accel, (uint32_t)motor1_target_speed,
                                               motor2_accel, (uint32_t)motor2_target_speed);
 
   if (success) {
     // Serial.println("MIXEDSPEED2ACCEL command successful."); // Don't print every time in loop
   } else {
     Serial.println("MIXEDSPEED2ACCEL command failed.");
     Serial.println("Check wiring, power, address, baud rate, and controller configuration (Is Velocity PID enabled for both motors?).");
   }
 
   // Change the target speeds periodically
   static unsigned long lastChange = 0;
   if (millis() - lastChange > 6000) { // Change every 6 seconds
       lastChange = millis();
       // Swap directions and potentially values
       int32_t temp_speed1 = motor1_target_speed;
       int32_t temp_speed2 = motor2_target_speed;
       uint32_t temp_accel1 = motor1_accel;
       uint32_t temp_accel2 = motor2_accel;
 
 
       motor1_target_speed = -temp_speed1; // Reverse M1
       motor2_target_speed = -temp_speed2; // Reverse M2
       // Optionally swap accels as well
       // motor1_accel = temp_accel2;
       // motor2_accel = temp_accel1;
 
 
       Serial.print("Switching M1 Accel/Speed: "); Serial.print(motor1_accel); Serial.print("/"); Serial.print(motor1_target_speed);
       Serial.print(", M2 Accel/Speed: "); Serial.print(motor2_accel); Serial.print("/"); Serial.print(motor2_target_speed);
       Serial.println(" counts/sec");
   }
 
   // Short delay to prevent flooding serial
   delay(50);
 }