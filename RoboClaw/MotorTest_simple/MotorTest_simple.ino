// Include the RoboClaw library
#include <RoboClaw.h>
#include <Basicmicro.h>
#include <SoftwareSerial.h>
// Create the RoboClaw object, passing the pointer to the hardware serial object
// and the serial timeout value
SoftwareSerial serial(10, 11);
Basicmicro roboclaw(&Serial, 10000);
#define address 0x80 

void setup() {
// Begin serial communication at the given baudrate
Serial.begin(115200);
roboclaw.begin(38400);
}

void loop()
{
// Call a method of RoboClaw object to control the motor controller
roboclaw.ForwardM1(address, 127);
delay(2000);
roboclaw.ForwardM1(address, 0);
delay(2000);
}