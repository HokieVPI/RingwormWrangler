#include <SoftwareSerial.h>
#include <RoboClaw.h> 

// Global Variables 
SoftwareSerial serial(10, 11); // RX=10, TX=11
RoboClaw roboclaw(&serial, 10000);
#define address 0x80 
uint32_t accel = 10000; // acceleration in counts/s^2
// static constexpr int Encoder_CPR = 300; // count per rev of encoder
static constexpr int left_QPPS = 42570;
static constexpr int right_QPPS = 44220;  
static constexpr float Kp = 1.54;
static constexpr float Ki = 0.24;
static constexpr float Kd = 0;
// static const float leftMotor = 3.0f;// rad/s
// static const float rightMotor = 3.0f;// rad/s
static const float runSpeed  = 0.30f; // 30% of max QPPS
static const float MaxOmega = 10.0f; // max rotational speed in rad/s

//---------- start Roboclaw functions----------//
// int32_t radPerSecToQPPS(float radPerSec){
//     // counts/sec = (rad/sec) * (counts/rev) / (2*pi rad/rev)
//     return (int32_t)(radPerSec * Encoder_CPR / (2.0f * PI));
//   }

//   // Print error flags, driver temp, main battery (and optional encoders/speeds)
// void printRoboClawStatus() {
//     uint32_t err = roboclaw.ReadError(address);
//     Serial.print("Err:0x");
//     Serial.print(err, HEX);
  
//     uint16_t temp;
//     if (roboclaw.ReadTemp(address, temp)) {
//       Serial.print(" T:");
//       Serial.print(temp / 10.0f);
//       Serial.print("C");
//       Serial.print(" T:--");
//     }
  
//     uint16_t mainV = roboclaw.ReadMainBatteryVoltage(address);
//     Serial.print(" V:");
//     Serial.print(mainV / 10.0f);
//     Serial.println("V");
  
//     // Optional: encoders and speeds
//     uint32_t enc1, enc2;
//     if (roboclaw.ReadEncoders(address, enc1, enc2)) {
//       Serial.print("  Enc M1:");
//       Serial.print(enc1);
//       Serial.print(" M2:");
//       Serial.println(enc2);
//     }
//     uint32_t is1, is2;
//     if (roboclaw.ReadISpeeds(address, is1, is2)) {
//       Serial.print("  Speed M1:");
//       Serial.print((int32_t)is1);
//       Serial.print(" M2:");
//       Serial.println((int32_t)is2);
//     }
//   }

// Setup
void setup() {
Serial.begin(115200);
roboclaw.begin(38400);

// uint32_t qpps;
// qpps = radPerSecToQPPS(MaxOmega);

roboclaw.SetM1VelocityPID(address,Ki,Kp,Kd,left_QPPS);
roboclaw.SetM2VelocityPID(address,Ki,Kp,Kd,right_QPPS);

Serial.println("Setup good");
}
// void runPhase(uint32_t duration) {
//     uint32_t start = millis();
//     uint32_t lastPrint = 0;
//     while (millis() - start < duration) {
//         if (millis() - lastPrint >= 500) {
//             lastPrint = millis();
//             printRoboClawStatus();
//         }
//     }
// }

// // Loop 
// void loop() {
//     int32_t base       = radPerSecToQPPS(baseSpeed);
//     int32_t boosted    = radPerSecToQPPS(baseSpeed * turnBoost);

//     // Phase 1: Both motors forward for 2 sec
//     Serial.println("Phase 1: Forward");
//     roboclaw.SpeedAccelM1(address, accel, base);
//     roboclaw.SpeedAccelM2(address, accel, base);
//     runPhase(2000);

//     // Phase 2: Left motor 10% faster for 1 sec (turns right)
//     Serial.println("Phase 2: Left faster");
//     roboclaw.SpeedAccelM1(address, accel, boosted);
//     roboclaw.SpeedAccelM2(address, accel, base);
//     runPhase(1000);

//     // Phase 3: Right motor 10% faster for 1 sec (turns left)
//     Serial.println("Phase 3: Right faster");
//     roboclaw.SpeedAccelM1(address, accel, base);
//     roboclaw.SpeedAccelM2(address, accel, boosted);
//     runPhase(1000);

//     // Phase 4: Both motors same speed for 2 sec
//     Serial.println("Phase 4: Forward");
//     roboclaw.SpeedAccelM1(address, accel, base);
//     roboclaw.SpeedAccelM2(address, accel, base);
//     runPhase(2000);

//     // Phase 5: Stop
//     Serial.println("Stopped.");
//     roboclaw.SpeedM1(address, 0);
//     roboclaw.SpeedM2(address, 0);

//     while (true) {} // halt
// }


void loop() {
    static uint32_t lastPrint = 0;
    int32_t leftBase = (int32_t)(left_QPPS  * runSpeed);
    int32_t rightBase = (int32_t)(right_QPPS * runSpeed);
    // int32_t leftQPPS  = radPerSecToQPPS(leftMotor);
    // int32_t rightQPPS = radPerSecToQPPS(rightMotor);
  Serial.println("predrive");
    roboclaw.SpeedAccelM1(address, accel, leftBase);
    roboclaw.SpeedAccelM2(address, accel, rightBase);
  
    // if (millis() - lastPrint >= 500) {
    //   lastPrint = millis();
    //   printRoboClawStatus();
    // }
    // Serial.println("postdrive");
    delay(4000);
    // roboclaw.SpeedAccelM1(address,accel, 0);
    // roboclaw.SpeedAccelM2(address,accel, 0);
    //   Serial.println("stop");
    // delay(1000);
  }




