#include <PortentaUWBShield.h>
#include <math.h>

#include <Basicmicro.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif


/**
 * pure pursuit path following algorithm for a single tag
 working on implementing custom pure pursuit path following algorithm

 used cursor--- need to verify code 
 **/

// In line of sight 
float insight_A1;
float insight_A2;
float insight_A3;
// Anchor Locations in Centimeters (x,y) z=0 

const float Anchor1_x=0;// cm 
const float Anchor1_y=1273.45; // cm  41.78 ft 
const float Anchor2_x=1280.16; // cm  42.23ft 
const float Anchor2_y=0; // cm 
const float Anchor3_x=2090.01; // cm 
const float Anchor3_y=1125.32; // cm 36.92 ft 

// const float Anchor1_x=0.5;// cm 
// const float Anchor1_y=1271.02; // cm 
// const float Anchor2_x=1280.16; // cm 
// const float Anchor2_y=0.5; // cm 
// const float Anchor3_x=2133.60; // cm 
// const float Anchor3_y=1158.24; // cm 

// const float Anchor1_x=0;// cm 
// const float Anchor1_y=0; // cm 
// const float Anchor2_x=719; // cm 
// const float Anchor2_y=0; // cm 
// const float Anchor3_x=320; // cm 
// const float Anchor3_y=752; // cm 

// Initialize Distance Variables
int dist_1 = 0;
int dist_2 = 0;
int dist_3 = 0; 
// Anchor received state variables
bool anchor1_received = false;
bool anchor2_received = false;
bool anchor3_received = false;
//previous position state variable 
double volatile currentX_global = 0.0f;// cm 
double volatile currentY_global = 0.0f;// cm 
bool prev_valid = false;

// Heading State Variable 
double Azimuth = 0.0f; // rad
double volatile global_azimuth = 0.0f; // rad

// constants 
static constexpr int HALF_CIRCULAR_BUFFER_SIZE = 5 ;
static constexpr int CIRCULAR_BUFFER_SIZE = HALF_CIRCULAR_BUFFER_SIZE*2;
const float MinMovement = 0.5f; // cm 
const float minMovement_sq=MinMovement*MinMovement; // minimum movement squared
static int staleCount = 0;
const int MAX_STALE = 5;
float x_circular_buffer[CIRCULAR_BUFFER_SIZE];
float y_circular_buffer[CIRCULAR_BUFFER_SIZE];
int head_index = 0;
int tail_index = CIRCULAR_BUFFER_SIZE-1;
volatile bool inRangingHandler = false;
// pure pursuit variables & constants 
// waypoint constant
static constexpr int waypoint_radius = 50; // cm
static constexpr float look_ahead=200.0f; // cm 
volatile bool newPosition = false;
float delta_x; // differnce in look-ahead distance from current position  
float delta_y; // differnce in look-ahead distance from current position 
float L_d; // Look-Ahead Distance in cm 
float L_d2; // Look-Ahead Distance squared 
float K; // Curvature Coeff (K)
float omega; // Rotational Velocity in rad/s
const float velocity = 50.0f;  // Constant Velocity in cm/s
static constexpr int wheelRadius = 15;  //cm 
static constexpr float trackWidth =43.18;  // Wheel to Wheel in cm 
float leftMotor; 
float rightMotor; 

// RoboClaw UART on Portenta C33: TX = pin 14, RX = pin 13
UART controllerSerial(14, 13);

#define MOTOR_ADDRESS        128
#define LIBRARY_READ_TIMEOUT 10000

static constexpr float ENCODER_CPR   = -24293.0f;
static constexpr float RAD_TO_COUNTS = ENCODER_CPR / (2.0f * PI);
uint32_t motor_accel = 25000;

Basicmicro controller(&controllerSerial, LIBRARY_READ_TIMEOUT);

// ----------- Functions----------//

//---------- start waypoint handling functions----------//

 // set up waypoint struct 
 struct Waypoint {
  float wp_x;
  float wp_y; 
}; 
// goal result struct
struct GoalResult {
  float gx;      // goal x in cm
  float gy;      // goal y in cm
  bool  found;   // true if circle intersected the path
};
//  establish path length and waypoints
static constexpr int PATH_LENGTH = 4;
static Waypoint path[PATH_LENGTH] = {
  {890,308},
  {890,1097},{1200,1097},
  {1200,308}
};
// functions to get waypoint x and y coordinates and path length
float getWaypointX(int j){
  return (j>=0 && j<PATH_LENGTH) ? path[j].wp_x : 0.0f;
}
float getWaypointY(int j){
  return (j>=0 && j<PATH_LENGTH) ? path[j].wp_y : 0.0f;
}

int getPathLength(){
  return PATH_LENGTH;
}
// establish path state 
static int pathSegIdx = 0; // current path segment index

void advancePathSegment(){
  if(pathSegIdx < PATH_LENGTH - 1){
    pathSegIdx++;
  }

}
int getCurrentPathSegmentIndex(){
  return pathSegIdx;
}
bool PathComplete(){
  return
   pathSegIdx == PATH_LENGTH - 1;
}

void AdvancePathSegment(){
  if(!PathComplete()){

    int nextWaypoint=pathSegIdx+1;
    float deltaX=getWaypointX(nextWaypoint)-currentX_global;
    float deltaY=getWaypointY(nextWaypoint)-currentY_global;
    float distanceToNextWaypoint_sq=deltaX*deltaX+deltaY*deltaY;

    if (distanceToNextWaypoint_sq <= waypoint_radius * waypoint_radius){
      advancePathSegment();
    }
    // could add in something to handle overshoot of waypoint radius
  }
}
//----------end waypoint handling functions----------//

//---------- start pure pursuit functions----------//


// Find intersection of lookahead circle with the path ahead.
// Uses: currentX_global, currentY_global, look_ahead, path[], pathSegIdx, PATH_LENGTH
GoalResult findLookaheadGoal() {
  GoalResult result;
  result.found = false;
  float Lsq = look_ahead * look_ahead;

  for (int seg = pathSegIdx; seg < PATH_LENGTH - 1; seg++) {
    float dsx = path[seg + 1].wp_x - path[seg].wp_x;
    float dsy = path[seg + 1].wp_y - path[seg].wp_y;

    float fx = path[seg].wp_x - (float)currentX_global;
    float fy = path[seg].wp_y - (float)currentY_global;

    float qa = dsx * dsx + dsy * dsy;
    float qb = 2.0f * (fx * dsx + fy * dsy);
    float qc = (fx * fx + fy * fy) - Lsq;

    float discriminant = qb * qb - 4.0f * qa * qc;

    if (discriminant < 0.0f) continue;

    float sqrtDisc = sqrtf(discriminant);

    float t1 = (-qb - sqrtDisc) / (2.0f * qa);
    float t2 = (-qb + sqrtDisc) / (2.0f * qa);

    float bestT = -1.0f;
    if (t2 >= 0.0f && t2 <= 1.0f) {
      bestT = t2;
    } else if (t1 >= 0.0f && t1 <= 1.0f) {
      bestT = t1;
    }

    if (bestT >= 0.0f) {
      result.gx = path[seg].wp_x + bestT * dsx;
      result.gy = path[seg].wp_y + bestT * dsy;
      result.found = true;
      return result;
    }
  }

  // Fallback: no intersection found, aim at next waypoint directly
  int nextWp = (pathSegIdx < PATH_LENGTH - 1) ? pathSegIdx + 1 : PATH_LENGTH - 1;
  result.gx = path[nextWp].wp_x;
  result.gy = path[nextWp].wp_y;
  result.found = true;
  return result;
}

//----------end pure pursuit functions----------//


// Helper to wrap angle to [-PI, PI]
static float wrapAnglePi(float a) {
  while (a > PI)  a -= 2.0f * PI;
  while (a < -PI) a += 2.0f * PI;
  return a;
}
// Helper to convert radians to degrees
float radiansToDegrees(float a) {
  return a * (180.0/PI);
}


// handler for ranging notifications
void rangingHandler(UWBRangingData &rangingData) {
  inRangingHandler = true;
  if(rangingData.measureType()==(uint8_t)uwb::MeasurementType::TWO_WAY)
  {

    RangingMeasures twr=rangingData.twoWayRangingMeasure();


  for (int j = 0; j < rangingData.available(); j++) {
    // skip invalid measurements
    if (twr[j].status != 0 || twr[j].distance == 0xFFFF) {
      continue;
    }
  
    // classify by short MAC (first two bytes)
    if (twr[j].peer_addr[0] == 0x22 && twr[j].peer_addr[1] == 0x22) {
      dist_1 = twr[j].distance;
      anchor1_received = true;
      //  insight_A3=twr[j].nlos;
          // Serial.println(insight_A1);
      // Serial.println(dist_1);
    } else if (twr[j].peer_addr[0] == 0x33 && twr[j].peer_addr[1] == 0x33) {
      dist_2 = twr[j].distance;
      anchor2_received = true;
      // insight_A2=twr[j].nlos;
        //  Serial.println(insight_A2);
      // Serial.println(dist_2);
      } else if (twr[j].peer_addr[0] == 0x44 && twr[j].peer_addr[1] == 0x44) {
      dist_3 = twr[j].distance;
      //  insight_A3=twr[j].nlos;
      anchor3_received = true;
        //  Serial.println(insight_A3);
      // Serial.println(dist_3);
    }
  }
  
if (!anchor1_received || !anchor2_received || !anchor3_received) {
  // Serial.println("bad anchor connection");
    inRangingHandler = false;
  return;
}else 
{
  // reseting bools
  anchor1_received = false;
  anchor2_received = false;
  anchor3_received = false;

  float A = 2.0f*Anchor2_x - 2.0f*Anchor1_x; 
  float B = 2.0f*Anchor2_y - 2.0f*Anchor1_y; 
  float C = dist_1*dist_1 - dist_2*dist_2 - Anchor1_x*Anchor1_x + Anchor2_x*Anchor2_x - Anchor1_y*Anchor1_y + Anchor2_y*Anchor2_y; 
  float D = 2.0f*Anchor3_x - 2.0f*Anchor2_x;
  float E = 2.0f*Anchor3_y - 2.0f*Anchor2_y; 
  float F = dist_2*dist_2 - dist_3*dist_3 - Anchor2_x*Anchor2_x + Anchor3_x*Anchor3_x - Anchor2_y*Anchor2_y + Anchor3_y*Anchor3_y;
 
  float det = A*E - B*D;

  if (fabsf(det) < 1e-6) {
    Serial.println("Error: Anchors are collinear, cannot calculate position");
    inRangingHandler = false;
    return;
  }

  float x = (C*E - F*B) / det;
  float y = (A*F - C*D) / det;
Serial.println(x);
Serial.println(y);
  if(!prev_valid) {
    for(int i = 0; i < CIRCULAR_BUFFER_SIZE; i++) {
      x_circular_buffer[i] = x;
      y_circular_buffer[i] = y;
    }
    prev_valid = true;
    inRangingHandler = false;
    return;
  }
// ------------ Circular Buffer Advance ------------ //
// Advance Circular Buffer 
  head_index++;
  if (head_index == CIRCULAR_BUFFER_SIZE) {
    head_index = 0;
  }
  tail_index++;
  if (tail_index == CIRCULAR_BUFFER_SIZE) {
    tail_index = 0;
  }
  x_circular_buffer[head_index] = x;
  y_circular_buffer[head_index] = y;
// ------------ End Circular Buffer Advance ------------ //


float currentX=0.0f;
float currentY=0.0f;
float prevX=0.0f;
float prevY=0.0f;
// ------------ Weighted Average ------------ //
  float weights[HALF_CIRCULAR_BUFFER_SIZE];
  for (int i = 0; i < HALF_CIRCULAR_BUFFER_SIZE; i++) {
    weights[i] = 1.0f/HALF_CIRCULAR_BUFFER_SIZE;
  }
   weights[0] = 0.3;
   weights[1] = 0.25;
   weights[2] = 0.2;
   weights[3] = 0.15;
   weights[4] = 0.1;
  int index = head_index;

  for (int i = 0; i < HALF_CIRCULAR_BUFFER_SIZE; i++) {
    if (index < 0) {
      index = index + CIRCULAR_BUFFER_SIZE;
    }
    currentX += weights[i]*x_circular_buffer[index];
    currentY += weights[i]*y_circular_buffer[index];
    index--;
  }

  index = tail_index;
  for (int i = 0; i < HALF_CIRCULAR_BUFFER_SIZE; i++) {
    if (index >= CIRCULAR_BUFFER_SIZE) {
      index = index - CIRCULAR_BUFFER_SIZE;
    }
    prevX += weights[i]*x_circular_buffer[index];
    prevY += weights[i]*y_circular_buffer[index];
    index++;
  }
    currentX_global = currentX;
    currentY_global = currentY; 
  // Serial.print("( ");
  // Serial.println(currentX_global);
  // Serial.print(" , ");
  // Serial.println(currentY_global);
  // Serial.print(") ");
// ------------ End Weighted Average ------------ //

// ------------ Heading Calculation ------------ //
// Calculate the differnce in position from previous point  
  float dx = currentX - prevX;
  float dy = currentY - prevY;
// Compare the dx^2+dy^2 to the distance to flag invalid headings
  float dist_sq = dx*dx + dy*dy;


// If the distance is greater than the minimum movement, calculate the azimuth
  if (dist_sq >= minMovement_sq) {
    Azimuth = atan2f(dy, dx);
    global_azimuth = Azimuth;
    staleCount = 0;
  }else{
    // Serial.print("Azimuth invalid  ");
    staleCount++;
    if(staleCount >= MAX_STALE) {
      // Moves the robot forward when min movement is not met
      float minDrive=0.5f*velocity;// cm/s
      float minMotorCmd=minDrive/wheelRadius;
      int32_t minCounts=(int32_t)(minMotorCmd*RAD_TO_COUNTS);
      controller.SpeedAccelM1M2_2(MOTOR_ADDRESS,
                                   motor_accel, (int32_t)(-minCounts),
                                   motor_accel, (int32_t)(-minCounts));
    }

  }

  newPosition = true;
// ------------ End Heading Calculation ------------ //
}
}

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

  controller.begin(38400);
  delay(100);
  controller.SetM1VelocityPID(MOTOR_ADDRESS, 1.79279, 0.27940, 0.00000, 70620);
  controller.SetM2VelocityPID(MOTOR_ADDRESS, 1.74675, 0.26201, 0.00000, 69630);

}

void loop() {
  #if defined(ARDUINO_PORTENTA_C33)
  /* Only the Portenta C33 has an RGB LED. */
  digitalWrite(LEDR, !digitalRead(LEDR));
#endif
delay(10);
  while (inRangingHandler || !newPosition) {
    delay(10);
  }
  newPosition = false;  // consumed; wait for next update before next iteration
  AdvancePathSegment(); // check if we reached the next waypoint
  if (PathComplete()) {
    controller.SpeedAccelM1M2_2(MOTOR_ADDRESS,
                                 motor_accel, 0,
                                 motor_accel, 0);
    Serial.println("Path Complete");
    inRangingHandler = false;
    return;
  }

  GoalResult goal = findLookaheadGoal();

  delta_x = goal.gx - currentX_global;
  delta_y = goal.gy - currentY_global;

  float angleToGoal = atan2f(delta_y, delta_x);
  // Serial.print("Goal xy: ");
  // Serial.println(goal.gx);
  // Serial.println(goal.gy);
  float DesiredHeading = radiansToDegrees(angleToGoal);
  // Serial.print("Desired Heading: ");
  Serial.println(DesiredHeading);
  float azimuth_deg = radiansToDegrees(global_azimuth);
  Serial.println(azimuth_deg);
  // Serial.println(currentX_global);
  // Serial.println(currentY_global);

  L_d2 = delta_x * delta_x + delta_y * delta_y;
  L_d = sqrtf(L_d2);

  float alpha = wrapAnglePi(angleToGoal - global_azimuth);

  if (L_d < 1.0f) {
    K = 0.0f;
    controller.SpeedAccelM1M2_2(MOTOR_ADDRESS,
                                 motor_accel, 0,
                                 motor_accel, 0);
  } else {
    K = 2.0f * sinf(alpha) / L_d;
    omega = K * velocity;
    leftMotor  = (velocity - omega * trackWidth / 2.0f) / wheelRadius;
    rightMotor = (velocity + omega * trackWidth / 2.0f) / wheelRadius;
// Serial.println(leftMotor);
// Serial.println(rightMotor);
    int32_t leftCounts  = (int32_t)(leftMotor  * RAD_TO_COUNTS);
    int32_t rightCounts = (int32_t)(rightMotor * RAD_TO_COUNTS);

    const int32_t MAX_MOTOR_COUNTS = 70000;
    leftCounts  = constrain(leftCounts,  -MAX_MOTOR_COUNTS, MAX_MOTOR_COUNTS);
    rightCounts = constrain(rightCounts, -MAX_MOTOR_COUNTS, MAX_MOTOR_COUNTS);

    controller.SpeedAccelM1M2_2(MOTOR_ADDRESS,
                                 motor_accel, (uint32_t)rightCounts,
                                 motor_accel, (uint32_t)leftCounts);
  }
}