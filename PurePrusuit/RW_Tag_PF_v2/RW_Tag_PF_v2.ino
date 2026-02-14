#include <PortentaUWBShield.h>
#include <math.h>
#include <RoboClaw.h>


/**
 * pure pursuit path following algorithm for a single tag
 working on implementing custom pure pursuit path following algorithm

 used cursor--- need to verify code 
 **/



// Anchor Locations in Centimeters (x,y) z=0 
const float Anchor1_x=1290;// cm 
const float Anchor1_y=0; // cm 
const float Anchor2_x=0; // cm 
const float Anchor2_y=1262; // cm 
const float Anchor3_x=1379; // cm 
const float Anchor3_y=2641; // cm 
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
static constexpr int HALF_CIRCULAR_BUFFER_SIZE = 5;
static constexpr int CIRCULAR_BUFFER_SIZE = HALF_CIRCULAR_BUFFER_SIZE*2;
const float MinMovement = 5.0f; // cm 
const float minMovement_sq=MinMovement*MinMovement; // minimum movement squared
float x_circular_buffer[CIRCULAR_BUFFER_SIZE];
float y_circular_buffer[CIRCULAR_BUFFER_SIZE];
int head_index = 0;
int tail_index = CIRCULAR_BUFFER_SIZE-1;
bool inRangingHandler = false;
// pure pursuit variables & constants 
// waypoint constant
static constexpr int waypoint_radius = 25; // cm
static constexpr float look_ahead=100.0f; // cm
bool newPosition = false;
float delta_x; // differnce in look-ahead distance from current position  
float delta_y; // differnce in look-ahead distance from current position 
float L_d; // Look-Ahead Distance in cm 
float L_d2; // Look-Ahead Distance squared 
float K; // Curvature Coeff (K)
float omega; // Rotational Velocity in rad/s
const float velocity = 25.0f;  // Constant Velocity in cm/s
static constexpr int wheelRadius = 15;  //cm 
static constexpr int trackWidth =86;  // Wheel to Wheel in cm 
float leftMotor; 
float rightMotor; 
// Roboclaw serial and constants 
RoboClaw roboclaw(&Serial1,10000);
#define address 0x80 
static constexpr int Encoder_CPR = 300; 
uint32_t accel = 10000; // acceleration in counts/s^2

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
  {200, 200},
  {792, 1321},
  {792, 1864},
  {792, 2441}
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

    if (discriminant < 0.0f) continue;  // circle misses this segment

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

  // Fallback: no intersection found, aim at next waypoint directly
  if (!result.found) {
    int nextWp = (pathSegIdx < PATH_LENGTH - 1) ? pathSegIdx + 1 : PATH_LENGTH - 1;
    result.gx = path[nextWp].wp_x;
    result.gy = path[nextWp].wp_y;
    result.found = true;
  }

  return result;
}
//----------end pure pursuit functions----------//

//---------- start Roboclaw functions----------//
int32_t radPerSecToQPPS(float radPerSec){
  // counts/sec = (rad/sec) * (counts/rev) / (2*pi rad/rev)
  return (int32_t)(radPerSec * Encoder_CPR / (2.0f * PI));
}
//----------end Roboclaw functions----------//
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
    } else if (twr[j].peer_addr[0] == 0x33 && twr[j].peer_addr[1] == 0x33) {
      dist_2 = twr[j].distance;
      anchor2_received = true;
      } else if (twr[j].peer_addr[0] == 0x44 && twr[j].peer_addr[1] == 0x44) {
      dist_3 = twr[j].distance;
      anchor3_received = true;
    }
  }
  
if (!anchor1_received || !anchor2_received || !anchor3_received) {
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
  // weights[] = {0.3,0.25,0.2,0.15,0.1};
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
  // Serial.println(currentX);
  // Serial.print(" , ");
  // Serial.println(currentY);
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
  }else{
    // Serial.print("Azimuth invalid  ");
    inRangingHandler = false;
    return;
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

  roboclaw.begin(38400);

}

void loop() {
  #if defined(ARDUINO_PORTENTA_C33)
  /* Only the Portenta C33 has an RGB LED. */
  digitalWrite(LEDR, !digitalRead(LEDR));
#endif

  while(inRangingHandler || !newPosition) {
    
    delay(10);
  }
AdvancePathSegment(); //check if we reached the next waypoint
if (PathComplete()){
  roboclaw.SpeedAccelM1M2(address, accel, 0, 0);  // decelerate both motors to zero
    inRangingHandler = false;
  return;
}

GoalResult goal =findLookaheadGoal(); // compute goal point 


    delta_x=goal.gx-currentX_global; // differnce from goal point to current position 
    delta_y=goal.gy-currentY_global; // differnce from goal point to current position
// Find Look-ahead Distance and heading to goal
float angleToGoal = atan2f(delta_y, delta_x); // radians
// For debug, convert desired heading to degrees
float DesiredHeading = radiansToDegrees(angleToGoal);
Serial.print("Desired Heading: ");
Serial.println(DesiredHeading);
float GlobalHeading = radiansToDegrees(global_azimuth);
Serial.print("Global Heading: ");
Serial.println(GlobalHeading);
  Serial.println(currentX_global);
  Serial.println(currentY_global);

// Look-ahead distance from current point 
L_d2=delta_x*delta_x+delta_y*delta_y;  // Look-ahead Distance Squared 
L_d=sqrt(L_d2); // Look-ahead distance

// Compute the angle difference (alpha) in radians
float alpha = wrapAnglePi(angleToGoal - global_azimuth);

// Compute the Curvature Coeff (K)
if (L_d < 1.0f) {
  K=0.0f;
}else{
K=2.0f*sinf(alpha)/L_d; 

Serial.print("Curvature Coeff: ");
Serial.println(K);
omega=K*velocity; 

//Find Motor Velocities 
leftMotor=(velocity-omega*trackWidth/2.0f)/wheelRadius; 
rightMotor=(velocity+omega*trackWidth/2.0f)/wheelRadius; 

// convert rad/s to counts/s
int32_t leftQPPS = radPerSecToQPPS(leftMotor);
int32_t rightQPPS = radPerSecToQPPS(rightMotor);
// Debug output
    Serial.print("Left QPPS: ");
    Serial.println(leftQPPS);
    Serial.print("Right QPPS: ");
    Serial.println(rightQPPS);

//send to roboclaw with acceleration ramp 
roboclaw.SpeedAccelM1M2(address,accel,leftQPPS,rightQPPS);

}
}