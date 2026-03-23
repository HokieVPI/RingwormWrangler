/**
 * @file RCPulseMixedExample.ino
 * @brief Example for controlling a Basicmicro controller with RC pulse signals in mixed mode.
 * 
 * This example demonstrates how to generate RC servo pulse signals to control
 * a Basicmicro motor controller in RC mixed mode, which is typically used for
 * differential drive robots. The Servo library is used to generate the timing signals.
 * 
 * @note This requires the controller to be configured for RC pulse input and mixed mode.
 * 
 * @author Basicmicro
 * @date 2025-04-22
 */

#include <Servo.h> 

Servo myservo1;  // create servo object to control a Roboclaw channel (throttle)
Servo myservo2;  // create servo object to control a Roboclaw channel (steering)

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
 * Cycles through various control combinations for mixed mode RC control:
 * 1. Forward (throttle only)
 * 2. Backward (throttle only)
 * 3. Left turn (steering only)
 * 4. Right turn (steering only)
 * 5. Forward + Left (mixed)
 * 6. Forward + Right (mixed)
 * 7. Backward + Left (mixed)
 * 8. Backward + Right (mixed)
 * Each state is held for 2 seconds.
 * 
 * @note RC pulse values:
 * - 1500µs = neutral/center
 * - >1500µs = forward/right
 * - <1500µs = backward/left
 */
void loop() {
  //forward
  myservo1.writeMicroseconds(1600);  // throttle forward
  myservo2.writeMicroseconds(1500);  // steering center
  delay(2000);

  //backward
  myservo1.writeMicroseconds(1400);  // throttle backward
  myservo2.writeMicroseconds(1500);  // steering center
  delay(2000);

  //left
  myservo1.writeMicroseconds(1500);  // throttle center
  myservo2.writeMicroseconds(1600);  // steering left
  delay(2000);
  
  //right
  myservo1.writeMicroseconds(1500);  // throttle center
  myservo2.writeMicroseconds(1400);  // steering right
  delay(2000);
  
  //mixed forward/left
  myservo1.writeMicroseconds(1600);  // throttle forward
  myservo2.writeMicroseconds(1600);  // steering left
  delay(2000);

  //mixed forward/right
  myservo1.writeMicroseconds(1600);  // throttle forward
  myservo2.writeMicroseconds(1400);  // steering right
  delay(2000);
  
  //mixed backward/left
  myservo1.writeMicroseconds(1400);  // throttle backward
  myservo2.writeMicroseconds(1600);  // steering left
  delay(2000);

  //mixed backward/right
  myservo1.writeMicroseconds(1400);  // throttle backward
  myservo2.writeMicroseconds(1400);  // steering right
  delay(2000);
}