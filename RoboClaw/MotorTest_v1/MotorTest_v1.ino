#include <RoboClaw.h> 

// Global Variables 
RoboClaw roboclaw(&Serial1,10000); // initialize the roboclaw object (10000 is the timeout)
#define address 0x80 
uint32_t accel = 600; // acceleration in counts/s^2
static constexpr int Encoder_CPR = 300; // count per rev of encoder 
static constexpr float Kp = 1.0;
static constexpr float Ki = 0.5;
static constexpr float Kd = 0.25;
static const float leftMotor = 3.0f;// rad/s
static const float rightMotor = 3.0f;// rad/s
static const float MaxOmega = 10.0f; // max rotational speed in rad/s

//---------- start Roboclaw functions----------//
int32_t radPerSecToQPPS(float radPerSec){
    // counts/sec = (rad/sec) * (counts/rev) / (2*pi rad/rev)
    return (int32_t)(radPerSec * Encoder_CPR / (2.0f * PI));
  }

  // Print error flags, driver temp, main battery (and optional encoders/speeds)
void printRoboClawStatus() {
    uint32_t err = roboclaw.ReadError(address);
    Serial.print("Err:0x");
    Serial.print(err, HEX);
  
    uint16_t temp;
    if (roboclaw.ReadTemp(address, temp)) {
      Serial.print(" T:");
      Serial.print(temp / 10.0f);
      Serial.print("C");
      Serial.print(" T:--");
    }
  
    uint16_t mainV = roboclaw.ReadMainBatteryVoltage(address);
    Serial.print(" V:");
    Serial.print(mainV / 10.0f);
    Serial.println("V");
  
    // Optional: encoders and speeds
    uint32_t enc1, enc2;
    if (roboclaw.ReadEncoders(address, enc1, enc2)) {
      Serial.print("  Enc M1:");
      Serial.print(enc1);
      Serial.print(" M2:");
      Serial.println(enc2);
    }
    uint32_t is1, is2;
    if (roboclaw.ReadISpeeds(address, is1, is2)) {
      Serial.print("  Speed M1:");
      Serial.print((int32_t)is1);
      Serial.print(" M2:");
      Serial.println((int32_t)is2);
    }
  }

// Setup
void setup() {
Serial.begin(115200);
roboclaw.begin(38400);

uint32_t qpps;
qpps = radPerSecToQPPS(MaxOmega);

roboclaw.SetM1VelocityPID(address,Ki,Kp,Kd,qpps);
roboclaw.SetM2VelocityPID(address,Ki,Kp,Kd,qpps);


}

// Loop 
void loop() {
    static uint32_t lastPrint = 0;
  
    int32_t leftQPPS  = radPerSecToQPPS(leftMotor);
    int32_t rightQPPS = radPerSecToQPPS(rightMotor);
  
    roboclaw.SpeedAccelM1(address, accel, leftQPPS);
    roboclaw.SpeedAccelM2(address, accel, rightQPPS);
  
    if (millis() - lastPrint >= 500) {
      lastPrint = millis();
      printRoboClawStatus();
    }
  
    delay(2000);
    roboclaw.SpeedM1(address, 0);
    roboclaw.SpeedM2(address, 0);
    delay(1000);
  }




