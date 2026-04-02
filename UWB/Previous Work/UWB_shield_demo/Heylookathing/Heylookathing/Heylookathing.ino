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

// Anchor Locations in Centimeters (x,y) z=0 

// const float Anchor1_x=1296.924;// cm 
// const float Anchor1_y=4.572; // cm 
// const float Anchor2_x=4.572; // cm 
// const float Anchor2_y=1141.781; // cm 
// const float Anchor3_x=2087.88; // cm 
// const float Anchor3_y=1264.92; // cm 
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
//previous position state variable 
double currentX_global = 0.0f;// cm 
double currentY_global = 0.0f;// cm 
bool prev_valid = false;

// Heading State Variable 
double Azimuth = 0.0f; // rad
double global_azimuth = 0.0f; // rad

// constants 
static constexpr int HALF_CIRCULAR_BUFFER_SIZE = 5 ;
static constexpr int CIRCULAR_BUFFER_SIZE = HALF_CIRCULAR_BUFFER_SIZE*2;
const float MinMovement = 2.0f; // cm 
const float minMovement_sq=MinMovement*MinMovement; // minimum movement squared
float x_circular_buffer[CIRCULAR_BUFFER_SIZE];
float y_circular_buffer[CIRCULAR_BUFFER_SIZE];
int head_index = 0;
int tail_index = CIRCULAR_BUFFER_SIZE-1;
volatile bool inRangingHandler = false;
// pure pursuit variables & constants 
// waypoint constant
static constexpr int waypoint_radius = 20; // cm
static constexpr float look_ahead=75.0f; // cm 
volatile bool newPosition = false;
float delta_x; // differnce in look-ahead distance from current position  
float delta_y; // differnce in look-ahead distance from current position 
float L_d; // Look-Ahead Distance in cm 
float L_d2; // Look-Ahead Distance squared 
float K; // Curvature Coeff (K)
float omega; // Rotational Velocity in rad/s
const float velocity = 25.0f;  // Constant Velocity in cm/s
static constexpr int wheelRadius = 15;  //cm 
static constexpr int trackWidth =43.18;  // Wheel to Wheel in cm 
float leftMotor; 
float rightMotor; 

// RoboClaw UART on Portenta C33: TX = pin 14, RX = pin 13
UART controllerSerial(14, 13);

#define MOTOR_ADDRESS        128
#define LIBRARY_READ_TIMEOUT 10000

static constexpr float ENCODER_CPR   = 24293.0f;
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
static constexpr int PATH_LENGTH = 6;
static Waypoint path[PATH_LENGTH] = {
  {244, 122},
  {244, 366},
  {295, 446},
  {382, 446},
  {457, 366},
  {457,122}
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

// advance when the tag is within the waypoint radius of the next waypoint
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

bool miss_wp;
miss_wp = true;

while (miss_wp)  {
  // Search each segment from pathSegIdx forward
  for (int seg = pathSegIdx; seg < PATH_LENGTH - 1; seg++) {

    // Segment endpoints A -> B from existing path[] array
    float dsx = path[seg + 1].wp_x - path[seg].wp_x;   // segment direction x
    float dsy = path[seg + 1].wp_y - path[seg].wp_y;   // segment direction y

    float fx = path[seg].wp_x - (float)currentX_global;  // segment start relative to robot x
    float fy = path[seg].wp_y - (float)currentY_global;  // segment start relative to robot y

    // Quadratic coefficients for circle-segment intersection
    float qa = dsx * dsx + dsy * dsy;
    float qb = 2.0f * (fx * dsx + fy * dsy);
    float qc = (fx * fx + fy * fy) - Lsq;

    float discriminant = qb * qb - 4.0f * qa * qc;

    if (discriminant < 0.0f){ 
      result.gx = path[seg].wp_x;
      result.gy = path[seg].wp_y;
      result.found = true;
      return result;  // first valid hit on the earliest forward segment
      miss_wp=false; // circle misses this segment
    } else{
    float sqrtDisc = sqrtf(discriminant);

    // Two candidate parameter values along the segment (0 = start, 1 = end)
    float t1 = (-qb - sqrtDisc) / (2.0f * qa);
    float t2 = (-qb + sqrtDisc) / (2.0f * qa);

    // Pick the largest valid t (furthest forward on segment)
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
      return result;  // first valid hit on the earliest forward segment
    }
  }
  }
  miss_wp=false;
  

  // Fallback: no intersection found, aim at next waypoint directly
  // could increase lookahead distance or 
  if (!result.found) {
    int nextWp = (pathSegIdx < PATH_LENGTH - 1) ? pathSegIdx + 1 : PATH_LENGTH - 1;
    result.gx = path[nextWp].wp_x;
    result.gy = path[nextWp].wp_y;
    result.found = true;
  }

  return result;
}
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
      Serial.println(dist_1);
    }
  }
  
if (!anchor1_received || !anchor2_received || !anchor3_received) {
  // Serial.print("bad anchor connection");
    inRangingHandler = false;
  return;
}else 
{
  // reseting bools
  anchor1_received = false;




// ------------ Circular Buffer Advance ------------ //
// Advance Circular Buffer 

// ------------ End Circular Buffer Advance ------------ //





  // Serial.print("( ");
  // Serial.println(currentX_global);
  // Serial.print(" , ");
  // Serial.println(currentY_global);
  // Serial.print(") ");
// ------------ End Weighted Average ------------ //

// ------------ Heading Calculation ------------ //
// Calculate the differnce in position from previous point  

// Compare the dx^2+dy^2 to the distance to flag invalid headings


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
  UWBMacAddress dstAddr1(UWBMacAddress::Size::SHORT,destination1);


  // Create a list of destination addresses
  UWBMacAddressList dest(UWBMacAddress::Size::SHORT);
  dest.add(dstAddr1);

 

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

  while (inRangingHandler || !newPosition) {
    delay(10);
  }
  newPosition = false;  // consumed; wait for next update before next iteration
  AdvancePathSegment(); // check if we reached the next waypoint
}