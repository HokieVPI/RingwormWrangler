/**
 * @file SimpleSerial.ino
 * @brief Example for basic serial communication with a Basicmicro controller.
 * 
 * This example demonstrates how to send simple serial commands directly
 * to a Basicmicro motor controller in simple serial mode (not packet serial).
 * This is the most basic form of communication with the controller.
 * 
 * @note This requires the controller to be configured for simple serial mode.
 * 
 * @author Basicmicro
 * @date 2025-04-22
 */

//Includes required for software serial
#include <SoftwareSerial.h>

//See limitations of Arduino SoftwareSerial
SoftwareSerial mySerial(10, 11);

/**
 * @brief Setup function runs once at startup
 * 
 * Initializes software serial communication with the controller.
 */
void setup() {
  mySerial.begin(38400);
}

/**
 * @brief Main program loop
 * 
 * Cycles through various simple serial command combinations.
 * In simple serial mode, commands are single bytes sent directly to the controller:
 * - 0 = Stop
 * - 1-127 = Forward (1=slowest, 127=fastest)
 * - 128-255 = Reverse (255=slowest, 128=fastest) represented as -1 to -128
 * 
 * The first byte controls Motor 1, and the second byte controls Motor 2.
 */
void loop() {
  // Motor 1 very slow forward, Motor 2 maximum reverse
  mySerial.write(1);
  mySerial.write(-127);
  delay(2000);
  
  // Motor 1 half forward, Motor 2 no command
  mySerial.write(64);
  delay(1000);
  
  // Motor 1 maximum forward, Motor 2 very slow reverse
  mySerial.write(127);
  mySerial.write(-1);
  delay(2000);
  
  // Motor 1 half reverse, Motor 2 no command
  mySerial.write(-64);
  delay(1000);
  
  // Motor 1 very slow forward, Motor 2 very slow reverse
  mySerial.write(1);
  mySerial.write(-1);
  delay(2000);
  
  // Motor 1 stop, Motor 2 no command
  mySerial.write((uint8_t)0);
  delay(1000);
  
  // Motor 1 maximum forward, Motor 2 maximum reverse
  mySerial.write(127);
  mySerial.write(-127);
  delay(2000);
  
  // Motor 1 stop, Motor 2 no command
  mySerial.write((uint8_t)0);
  delay(1000);
}