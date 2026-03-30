#include <PortentaUWBShield.h>
#include <math.h>

// Static UWB x,y test — motor driver not used
// #include <Basicmicro.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif


/**
 * pure pursuit path following algorithm for a single tag
 working on implementing custom pure pursuit path following algorithm

 used cursor--- need to verify code 
 **/

// Per-anchor NLOS flag from twr[j].nlos (PortentaUWBShield uwb_types.hpp twr_mesr)
uint8_t nlos_1 = 0;
uint8_t nlos_2 = 0;
uint8_t nlos_3 = 0;
// Anchor Locations in Centimeters (x,y) z=0 

// const float Anchor1_x=0;// cm 
// const float Anchor1_y=1273.45; // cm  41.78 ft 
// const float Anchor2_x=1280.16; // cm  42.23ft 
// const float Anchor2_y=0; // cm 
// const float Anchor3_x=2090.01; // cm 
// const float Anchor3_y=1125.32; // cm 36.92 ft 
//------- Replace with new anchor locations---------//
// const float Anchor1_x=0;// cm 
// const float Anchor1_y=617.22; // cm 
// const float Anchor2_x=1280.16; // cm 
// const float Anchor2_y=0; // cm 
// const float Anchor3_x=1708.41; // cm 
// const float Anchor3_y=1570.03; // cm 
//--------------------------------------------------//
const float Anchor1_x=0;// cm 
const float Anchor1_y=0; // cm 
const float Anchor2_x=719; // cm 
const float Anchor2_y=0; // cm 
const float Anchor3_x=320; // cm 
const float Anchor3_y=752; // cm 

// Initialize Distance Variables
int dist_1 = 0;
int dist_2 = 0;
int dist_3 = 0; 
// Anchor received state variables
bool anchor1_received = false;
bool anchor2_received = false;
bool anchor3_received = false;
// Pure pursuit / smoothing / motors — disabled for static UWB x,y test
// double volatile currentX_global = 0.0f;
// double volatile currentY_global = 0.0f;
// bool prev_valid = false;
// double Azimuth = 0.0f;
// double volatile global_azimuth = 0.0f;
// static constexpr int HALF_CIRCULAR_BUFFER_SIZE = 5;
// static constexpr int CIRCULAR_BUFFER_SIZE = HALF_CIRCULAR_BUFFER_SIZE * 2;
// const float MinMovement = 0.5f;
// const float minMovement_sq = MinMovement * MinMovement;
// static int staleCount = 0;
// const int MAX_STALE = 5;
// float x_circular_buffer[CIRCULAR_BUFFER_SIZE];
// float y_circular_buffer[CIRCULAR_BUFFER_SIZE];
// int head_index = 0;
// int tail_index = CIRCULAR_BUFFER_SIZE - 1;
volatile bool inRangingHandler = false;
// static constexpr int waypoint_radius = 50;
// static constexpr float look_ahead = 200.0f;
// static constexpr float MIN_LOOKAHEAD = 80.0f;
// float adaptiveLookahead = look_ahead;
// static constexpr float MIN_SPEED_SCALE = 0.2f;
// volatile bool newPosition = false;
// float delta_x, delta_y, L_d, L_d2, K, omega;
// const float velocity = 50.0f;
// static constexpr float wheelRadius = 15.24f;
// static constexpr float trackWidth = 43.18f;
// float leftMotor, rightMotor;
// UART controllerSerial(14, 13);
// #define MOTOR_ADDRESS        128
// #define LIBRARY_READ_TIMEOUT 10000
// static constexpr float ENCODER_CPR = -24293.0f;
// static constexpr float RAD_TO_COUNTS = ENCODER_CPR / (2.0f * PI);
// uint32_t motor_accel = 25000;
// Basicmicro controller(&controllerSerial, LIBRARY_READ_TIMEOUT);

// ----------- Functions----------//
// Waypoints, pure pursuit, driveMotors — disabled for static UWB x,y test (restore from git / uncomment globals + Basicmicro).

// handler for ranging notifications — static test: trilateration + Serial x,y only
void rangingHandler(UWBRangingData &rangingData) {
  inRangingHandler = true;

  if (rangingData.measureType() != (uint8_t)uwb::MeasurementType::TWO_WAY) {
    inRangingHandler = false;
    return;
  }

  RangingMeasures twr = rangingData.twoWayRangingMeasure();

  nlos_1 = 0;
  nlos_2 = 0;
  nlos_3 = 0;
  anchor1_received = false;
  anchor2_received = false;
  anchor3_received = false;

  for (int j = 0; j < rangingData.available(); j++) {
    if (twr[j].status != 0 || twr[j].distance == 0xFFFF) {
      continue;
    }
    if (twr[j].peer_addr[0] == 0x22 && twr[j].peer_addr[1] == 0x22) {
      dist_1 = twr[j].distance;
      nlos_1 = twr[j].nlos;
      anchor1_received = true;
    } else if (twr[j].peer_addr[0] == 0x33 && twr[j].peer_addr[1] == 0x33) {
      dist_2 = twr[j].distance;
      nlos_2 = twr[j].nlos;
      anchor2_received = true;
    } else if (twr[j].peer_addr[0] == 0x44 && twr[j].peer_addr[1] == 0x44) {
      dist_3 = twr[j].distance;
      nlos_3 = twr[j].nlos;
      anchor3_received = true;
    }
  }

  if (!anchor1_received || !anchor2_received || !anchor3_received) {
    inRangingHandler = false;
    return;
  }

  // Red in StaticUWBtest.m when nlos==0 on two or more anchors (same rule)
  int nlosZeroCount = (nlos_1 == 0) + (nlos_2 == 0) + (nlos_3 == 0);
  bool NoSight = (nlosZeroCount >= 2);

  float A = 2.0f * Anchor2_x - 2.0f * Anchor1_x;
  float B = 2.0f * Anchor2_y - 2.0f * Anchor1_y;
  float C = dist_1 * dist_1 - dist_2 * dist_2 - Anchor1_x * Anchor1_x + Anchor2_x * Anchor2_x
            - Anchor1_y * Anchor1_y + Anchor2_y * Anchor2_y;
  float D = 2.0f * Anchor3_x - 2.0f * Anchor2_x;
  float E = 2.0f * Anchor3_y - 2.0f * Anchor2_y;
  float F = dist_2 * dist_2 - dist_3 * dist_3 - Anchor2_x * Anchor2_x + Anchor3_x * Anchor3_x
            - Anchor2_y * Anchor2_y + Anchor3_y * Anchor3_y;

  float det = A * E - B * D;
  if (fabsf(det) < 1e-6f) {
    Serial.println("Error: Anchors are collinear, cannot calculate position");
    inRangingHandler = false;
    return;
  }

  float x = (C * E - F * B) / det;
  float y = (A * F - C * D) / det;

  // One line per fix for readmatrix: x y nlos1 nlos2 nlos3 NoSight(0/1)
  Serial.println(x);
  // Serial.print(' ');
  Serial.println(y);
  // Serial.print(' ');
  // Serial.print((int)nlos_1);
  // Serial.print(' ');
  // Serial.print((int)nlos_2);
  // Serial.print(' ');
  // Serial.print((int)nlos_3);
  // Serial.print(' ');
  // Serial.println(NoSight ? 1 : 0);

  inRangingHandler = false;
}

void setup() {

  Serial.begin(115200);

#if defined(ARDUINO_PORTENTA_C33)
  /* Only the Portenta C33 has an RGB LED. */
  pinMode(LEDR, OUTPUT);
  digitalWrite(LEDR, LOW);
#endif

  // Define the source (this device) MAC address using 2-bytes MAC
  uint8_t devAddr[]={0x11,0x11};
  UWBMacAddress srcAddr(UWBMacAddress::Size::SHORT,devAddr);

  // Define multiple destination MAC addresses (controlees)
  uint8_t destination1[]={0x22,0x22};
  uint8_t destination2[]={0x33,0x33};
  uint8_t destination3[]={0x44,0x44};


  UWBMacAddress dstAddr1(UWBMacAddress::Size::SHORT,destination1);
  UWBMacAddress dstAddr2(UWBMacAddress::Size::SHORT,destination2);
  UWBMacAddress dstAddr3(UWBMacAddress::Size::SHORT,destination3);


  // Create a list of destination addresses
  UWBMacAddressList dest(UWBMacAddress::Size::SHORT);
  dest.add(dstAddr1);
  dest.add(dstAddr2);
  dest.add(dstAddr3);
 

  // register the ranging notification handler before starting
  UWB.registerRangingCallback(rangingHandler);

  UWB.begin(); //start the UWB stack, use Serial for the log output
  Serial.println("Starting UWB ...");

  //wait until the stack is initialised
  while(UWB.state()!=0)
    delay(10);

  Serial.println("Starting multicast session ...");
  //setup a multicast session with ID 0x11223344
  UWBRangingOneToMany myController(0x11223344, srcAddr, dest);

  //add the session to the session manager, in case you want to manage multiple connections
  UWBSessionManager.addSession(myController);

  //prepare the session applying the default parameters
  myController.init();

  //start the session
  myController.start();

  // Static UWB x,y test — RoboClaw not initialized
  // controller.begin(38400);
  // delay(100);
  // controller.SetM1VelocityPID(MOTOR_ADDRESS, 1.79279, 0.27940, 0.00000, 70620);
  // controller.SetM2VelocityPID(MOTOR_ADDRESS, 1.74675, 0.26201, 0.00000, 69630);

}

void loop() {
  // Static UWB x,y test — work happens in rangingHandler; no path following
#if defined(ARDUINO_PORTENTA_C33)
  digitalWrite(LEDR, !digitalRead(LEDR));
#endif
  delay(50);
}