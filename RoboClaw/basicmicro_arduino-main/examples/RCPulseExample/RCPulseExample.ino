/**
 * @file RCPulseExample.ino
 * @brief Example for controlling a Basicmicro motor controller with RC pulse signals.
 * 
 * This example demonstrates how to generate RC servo pulse signals to control
 * a Basicmicro motor controller in RC mode. The Servo library is used to
 * generate the appropriate timing signals.
 * 
 * @note This requires the controller to be configured for RC pulse input mode.
 * 
 * @author Basicmicro
 * @date 2025-04-22
 */

#include <Servo.h> 
 
Servo myservo1;  // create servo object to control a Roboclaw channel
Servo myservo2;  // create servo object to control a Roboclaw channel
 
/**
 * @brief Setup function runs once at startup
 * 
 * Attaches the RC signal outputs to Arduino pins 5 and 6.
 */
void setup() {
  myservo1.attach(5);  // attaches the RC signal on pin 5 to the servo object 
  myservo2.attach(6);  // attaches the RC signal on pin 6 to the servo object 
} 
 
/**
 * @brief Main program loop
 * 
 * Alternately sends RC pulse signals to run motors in opposite directions:
 * - First cycle: Motor 1 full forward, Motor 2 full reverse
 * - Second cycle: Motor 1 full reverse, Motor 2 full forward
 * Each state is held for 2 seconds.
 * 
 * @note RC pulse values:
 * - 1500µs = neutral/stop
 * - 1250µs = full forward
 * - 1750µs = full reverse
 */
void loop() {
  // Motor 1 full forward, Motor 2 full reverse
  myservo1.writeMicroseconds(1250);  // full forward
  myservo2.writeMicroseconds(1750);  // full reverse
  delay(2000);
  
  // Motor 1 full reverse, Motor 2 full forward
  myservo1.writeMicroseconds(1750);  // full reverse
  myservo2.writeMicroseconds(1250);  // full forward
  delay(2000);
}