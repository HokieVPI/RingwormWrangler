#include <PortentaUWBShield.h>
#include <math.h>
#include <Basicmicro.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// placing other pins here
const int RoboClawFusePin = 3;
const int SolenoidFusePin = 2;
const int PumpFusePin = 1;
const int UltraSonicTriggerPin = 11;
const int UltraSonicEchoPin = 12;
// const int S1Pin = 13; // S1 used for roboclaw 
// const int S2Pin = 14; // S2



// orange, blue, white, green
// 0, 1, 2, 4


/**
 * pure pursuit path following algorithm for a single tag
 working on implementing custom pure pursuit path following algorithm

 used cursor--- need to verify code 
 **/

// In line of sight 
float insight_A1;
float insight_A2;
float insight_A3;
float insight_A4;
// Anchor Locations in Centimeters (x,y) z=0 

const float Anchor1_x=0;// cm 
const float Anchor1_y=1273.45; // cm  41.78 ft 
const float Anchor2_x=1280.16; // cm  42.23ft 
const float Anchor2_y=0; // cm 
const float Anchor3_x=2090.01; // cm 
const float Anchor3_y=1125.32; // cm 36.92 ft 
const float Anchor4_x=819.91; // cm  
const float Anchor4_y=2654.19; // cm 



// const float Anchor1_x=0;// cm 
// const float Anchor1_y=0; // cm 
// const float Anchor2_x=722; // cm 
// const float Anchor2_y=0; // cm 
// const float Anchor3_x=722; // cm 
// const float Anchor3_y=445; // cm 
// const float Anchor4_x=0; // cm  
// const float Anchor4_y=707; // cm 

static constexpr int NUM_ANCHORS = 4;
const float anchorX[NUM_ANCHORS] = {Anchor1_x, Anchor2_x, Anchor3_x, Anchor4_x};
const float anchorY[NUM_ANCHORS] = {Anchor1_y, Anchor2_y, Anchor3_y, Anchor4_y};

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
int dist_4 = 0;
int anchorDist[NUM_ANCHORS] = {0, 0, 0, 0};
// Anchor received state variables
bool anchor1_received = false;
bool anchor2_received = false;
bool anchor3_received = false;
bool anchor4_received = false;
bool anchorOk[NUM_ANCHORS] = {false, false, false, false};
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
const float MinMovement = 1.0f; // cm 
const float minMovement_sq=MinMovement*MinMovement; // minimum movement squared
static int staleCount = 0;
const int MAX_STALE = 10;
float x_circular_buffer[CIRCULAR_BUFFER_SIZE];
float y_circular_buffer[CIRCULAR_BUFFER_SIZE];
int head_index = 0;
int tail_index = CIRCULAR_BUFFER_SIZE-1;
volatile bool inRangingHandler = false;
// pure pursuit variables & constants 
float tRobot = 0.0f;
float tMin = tRobot+0.2f;
// waypoint constant
// for wrestling room
static constexpr int waypoint_radius = 150; // cm
static constexpr int final_waypoint_radius = 200; // cm (larger radius only for final waypoint)
static constexpr float look_ahead=175.0f; // cm

// for lab space
// static constexpr int waypoint_radius = 50; // cm
// static constexpr int final_waypoint_radius = 100; // cm (larger radius only for final waypoint)
// static constexpr float look_ahead=100.0f; // cm


static constexpr float MIN_SPEED_SCALE = 0.2f; // floor at 20% of max velocity
volatile bool newPosition = false;
float delta_x; // differnce in look-ahead distance from current position  
float delta_y; // differnce in look-ahead distance from current position 
float L_d; // Look-Ahead Distance in cm 
float L_d2; // Look-Ahead Distance squared 
float K; // Curvature Coeff (K)
float omega; // Rotational Velocity in rad/s

// for wrestling room
const float velocity = 70.0f;  // Constant Velocity in cm/s

// for lab space
// const float velocity = 50.0f;  // Constant Velocity in cm/s

static constexpr float wheelRadius = 15.24f;  //cm 
static constexpr float trackWidth =43.18f;  // Wheel to Wheel in cm 
float leftMotor; 
float rightMotor; 
// Linear Actuator and Pump condition
float CleaningStage = 0; 
float ActuatorDuration = 4000;  // in (ms)
float CurrentTime = 0;
float SprayInterval = 1000;

// RoboClaw UART on Portenta C33: TX = pin 14, RX = pin 13
UART controllerSerial(14, 13);

#define MOTOR_ADDRESS        128
#define LIBRARY_READ_TIMEOUT 10000

static constexpr float ENCODER_CPR   = 24293.0f;
static constexpr float RAD_TO_COUNTS = ENCODER_CPR / (2.0f * PI);
uint32_t motor_accel = 10000;

Basicmicro controller(&controllerSerial, LIBRARY_READ_TIMEOUT);

// Linear Actuator Motor Controller: Retract = pin 6, Extend = pin 7

const int RetractPin = 6;
const int ExtentPin = 7;
const int PumpPin = 1; // orange
const int SolenoidPin = 2; // green

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
//  {950,400},{950,800},{1300,800},{1300,500}

static constexpr int PATH_LENGTH = 14;
static Waypoint path[PATH_LENGTH] = {
  {315.0, 415.0},
  {315.0, 2405.5},
  {427.9, 2405.5},
  {427.9, 315.0},
  {610.8, 315.0},
  {610.8, 2405.5},
  {793.6, 2405.5},
  {793.6, 315.0},
  {976.5, 315.0},
  {976.5, 2405.5},
  {1159.4, 2405.5},
  {1159.4, 315.0},
  {737.0, 315.0},
  {315.0, 415.0}
};

  // // for wrestling room
  // {200, 502.4},{200, 2252.4},
  // {442.5, 2252.4}, {442.5, 200},
  // {792.5, 200},{792.5, 2252.4},
  // {1142.5, 2252.4}, {1142.5, 200}

  // for lab space
    //  {240, 146},
    //  {240, 386},
    //  {452, 386},
    //  {452, 146},
    //  {240, 146},

  // for 
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


// Pump on + solenoid off pressurizes bladder while spray-active (no timers).
// After PathComplete(), CleaningStage is incremented before SprayActive(); stage 1 is the first cleaning phase.
static bool sprayOutputsActive() {
  return CleaningStage == 0;
}

void applySprayOutputs() {
  if (sprayOutputsActive()) {
    digitalWrite(SolenoidPin, LOW);
    if (CurrentTime == 0) {
      CurrentTime = millis();
      digitalWrite(PumpPin, HIGH);
    } else if (millis() >= CurrentTime + SprayInterval) {
      if (digitalRead(PumpPin) == HIGH){
        digitalWrite(PumpPin, LOW);
        CurrentTime = millis();
      } else {
        digitalWrite(PumpPin, HIGH);
        CurrentTime = millis();
      }
    }
    
  } else {
    digitalWrite(PumpPin, LOW);
    digitalWrite(SolenoidPin, HIGH);
  }
}

void SprayActive() {
  applySprayOutputs();
}

void MopActive() {
  if (CleaningStage == 1) {
    digitalWrite(ExtentPin, HIGH);
    digitalWrite(RetractPin, LOW);
    delay(ActuatorDuration);
    digitalWrite(ExtentPin, LOW);

  } else if (CleaningStage == 2) {
    digitalWrite(ExtentPin, LOW);
    digitalWrite(RetractPin, HIGH);
    delay(ActuatorDuration);
    digitalWrite(RetractPin, LOW);

  } else {
    digitalWrite(ExtentPin, LOW);
    digitalWrite(RetractPin, LOW);
  }
}


// advance when the tag is within the waypoint radius of the next waypoint
void AdvancePathSegment(){
  if(!PathComplete()){

    int nextWaypoint=pathSegIdx+1;
    float deltaX=getWaypointX(nextWaypoint)-currentX_global;
    float deltaY=getWaypointY(nextWaypoint)-currentY_global;
    float distanceToNextWaypoint_sq=deltaX*deltaX+deltaY*deltaY;

    int r = (nextWaypoint == PATH_LENGTH - 1) ? final_waypoint_radius : waypoint_radius;
    if (distanceToNextWaypoint_sq <= (float)r * (float)r){
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

    float tMinLocal = 0.0f;
    if ((seg == pathSegIdx || seg == pathSegIdx + 1) && qa > 1e-6f) {
      float robX = (float)currentX_global - path[seg].wp_x;
      float robY = (float)currentY_global - path[seg].wp_y;
      float tRobotLocal = (robX * dsx + robY * dsy) / qa;
      if (tRobotLocal > 0.0f) tMinLocal = tRobotLocal;
    }

    float bestT = -1.0f;
    if (t2 >= tMinLocal && t2 <= 1.0f) {
      bestT = t2;
    } else if (t1 >= tMinLocal && t1 <= 1.0f) {
      bestT = t1;
    }

    if (bestT >= 0.0f) {
      result.gx = path[seg].wp_x + bestT * dsx;
      result.gy = path[seg].wp_y + bestT * dsy;
      result.found = true;
      return result;
    }
  }

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

void driveMotors(float leftRadPerSec, float rightRadPerSec) {
  const int32_t MAX_MOTOR_COUNTS = 70000;
  int32_t leftCounts  = constrain((int32_t)(leftRadPerSec  * RAD_TO_COUNTS), -MAX_MOTOR_COUNTS, MAX_MOTOR_COUNTS);
  int32_t rightCounts = constrain((int32_t)(rightRadPerSec * RAD_TO_COUNTS), -MAX_MOTOR_COUNTS, MAX_MOTOR_COUNTS);
  controller.SpeedAccelM1M2_2(MOTOR_ADDRESS,
                               motor_accel, (uint32_t)rightCounts,
                               motor_accel, (uint32_t)leftCounts);
}

// Trilateration using Cramer's rule on any 3 anchors (indices into anchorX/Y arrays).
// Returns true on success, writes result into *outX, *outY.
bool trilaterate(int i0, int i1, int i2,
                 const int dist[], float *outX, float *outY) {
  float A = 2.0f*(anchorX[i1] - anchorX[i0]);
  float B = 2.0f*(anchorY[i1] - anchorY[i0]);
  float C = (float)dist[i0]*dist[i0] - (float)dist[i1]*dist[i1]
          - anchorX[i0]*anchorX[i0] + anchorX[i1]*anchorX[i1]
          - anchorY[i0]*anchorY[i0] + anchorY[i1]*anchorY[i1];
  float D = 2.0f*(anchorX[i2] - anchorX[i1]);
  float E = 2.0f*(anchorY[i2] - anchorY[i1]);
  float F = (float)dist[i1]*dist[i1] - (float)dist[i2]*dist[i2]
          - anchorX[i1]*anchorX[i1] + anchorX[i2]*anchorX[i2]
          - anchorY[i1]*anchorY[i1] + anchorY[i2]*anchorY[i2];
  float det = A*E - B*D;
  if (fabsf(det) < 1e-6f) return false;
  *outX = (C*E - F*B) / det;
  *outY = (A*F - C*D) / det;
  return true;
}

// Priority-ordered 3-anchor combinations (best geometry first).
// Indices refer to anchorX/Y/anchorDist arrays: 0=A1, 1=A2, 2=A3, 3=A4.
// Reorder these based on field testing to put the best-spread combo first.
struct AnchorCombo { int a, b, c; };
static const AnchorCombo combos[4] = {
  {0, 1, 2},  // Anchors 1,2,3  (proven default)
  {0, 1, 3},  // Anchors 1,2,4
  {0, 2, 3},  // Anchors 1,3,4
  {1, 2, 3},  // Anchors 2,3,4
  
};

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
    } else if (twr[j].peer_addr[0] == 0x55 && twr[j].peer_addr[1] == 0x55) {
      dist_4 = twr[j].distance;
      //  insight_A4=twr[j].nlos;
      anchor4_received = true;
    }
  }
  
  anchorOk[0] = anchor1_received;
  anchorOk[1] = anchor2_received;
  anchorOk[2] = anchor3_received;
  anchorOk[3] = anchor4_received;
  anchorDist[0] = dist_1;
  anchorDist[1] = dist_2;
  anchorDist[2] = dist_3;
  anchorDist[3] = dist_4;

  int nValid = anchor1_received + anchor2_received
             + anchor3_received + anchor4_received;

  anchor1_received = false;
  anchor2_received = false;
  anchor3_received = false;
  anchor4_received = false;

  if (nValid < 3) {
    inRangingHandler = false;
    return;
  }

  float x = 0.0f, y = 0.0f;
  bool solved = false;
  for (int c = 0; c < 4; c++) {
    int ia = combos[c].a, ib = combos[c].b, ic = combos[c].c;
    if (anchorOk[ia] && anchorOk[ib] && anchorOk[ic]) {
      if (trilaterate(ia, ib, ic, anchorDist, &x, &y)) {
        solved = true;
        break;
      }
    }
  }

  if (!solved) {
    inRangingHandler = false;
    return;
  }

// Serial.println(x);
// Serial.println(y);
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
    // Serial.print("min move  ");
    staleCount++;
    if(staleCount >= MAX_STALE) {
      float minDrive = 0.5f * velocity / wheelRadius;
      driveMotors(-minDrive, -minDrive);
    }

  }

  newPosition = true;
// ------------ End Heading Calculation ------------ //
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
  uint8_t destination4[]={0x55,0x55};

  UWBMacAddress dstAddr1(UWBMacAddress::Size::SHORT,destination1);
  UWBMacAddress dstAddr2(UWBMacAddress::Size::SHORT,destination2);
  UWBMacAddress dstAddr3(UWBMacAddress::Size::SHORT,destination3);
  UWBMacAddress dstAddr4(UWBMacAddress::Size::SHORT,destination4);

  // Create a list of destination addresses
  UWBMacAddressList dest(UWBMacAddress::Size::SHORT);
  dest.add(dstAddr1);
  dest.add(dstAddr2);
  dest.add(dstAddr3);
  dest.add(dstAddr4);
 

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

  pinMode(PumpPin, OUTPUT);
  pinMode(SolenoidPin, OUTPUT);
  pinMode(RetractPin, OUTPUT);
  pinMode(ExtentPin, OUTPUT);
  pinMode(RoboClawFusePin, OUTPUT);
  digitalWrite(RoboClawFusePin, HIGH);
  digitalWrite(PumpPin, LOW);
  // digitalWrite(SolenoidPin, HIGH);

  // digitalWrite(ExtentPin, LOW); // to raise the mop 
  // digitalWrite(RetractPin, HIGH);
  // delay(ActuatorDuration);
  // digitalWrite(RetractPin, LOW);
}

void loop() {
  // digitalWrite(1, HIGH);
  // digitalWrite(0, HIGH);
  #if defined(ARDUINO_PORTENTA_C33)
  /* Only the Portenta C33 has an RGB LED. */
  digitalWrite(LEDR, !digitalRead(LEDR));
#endif
delay(10);
  while (inRangingHandler || !newPosition) {
    delay(10);
  }

  // Serial.println(pathSegIdx);
  newPosition = false;  // consumed; wait for next update before next iteration
  AdvancePathSegment(); // check if we reached the next waypoint
  // applySprayOutputs();  // hold pump/solenoid state for whole time CleaningStage == 1
  // digitalWrite(RoboClawFusePin, HIGH);
  if (PathComplete()) {
    // Advance cleaning stage
    CleaningStage = CleaningStage + 1;
    // does pass again
    controller.SpeedAccelM1M2(MOTOR_ADDRESS, motor_accel, 0, 0);
    delay(3000);
    pathSegIdx = 0;
    
    // SprayActive();
    // MopActive();
    // Insert code to start path following again
    if (CleaningStage == 2) {
      // digitalWrite(RoboClawFusePin, LOW);
      controller.SpeedAccelM1M2_2(MOTOR_ADDRESS,
                                   motor_accel, 0,
                                   motor_accel, 0);
      Serial.end();
      while (true) {
        delay(1000);
      }
      Serial.println("Path Complete");
      inRangingHandler = false;
      return;
    }
  }

  GoalResult goal = findLookaheadGoal();

  delta_x = goal.gx - currentX_global;
  delta_y = goal.gy - currentY_global;

  float angleToGoal = atan2f(delta_y, delta_x);
  // Serial.print("Goal xy: ");
  Serial.println(goal.gx);
  Serial.println(goal.gy);
  float DesiredHeading = radiansToDegrees(angleToGoal);
  Serial.println(DesiredHeading);
  float azimuth_deg = radiansToDegrees(global_azimuth);
  Serial.println(azimuth_deg);
  Serial.println(currentX_global);
  Serial.println(currentY_global);


  L_d2 = delta_x * delta_x + delta_y * delta_y;
  L_d = sqrtf(L_d2);

  float alpha = wrapAnglePi(angleToGoal - global_azimuth);

  float absAlpha = fabsf(alpha);
  // float speedScale = 0.5f;
  // if (absAlpha > PI / 4.0f) {
  //   speedScale = (PI - absAlpha) / PI;
  //   if (speedScale < MIN_SPEED_SCALE) speedScale = MIN_SPEED_SCALE;
  // }
   float cmdVelocity = velocity;

  if (L_d < 1.0f) {
    K = 0.0f;
    driveMotors(0, 0);
  } else {
    K = 2.0f * sinf(alpha) / L_d;
    omega = K * cmdVelocity;
    leftMotor  = (cmdVelocity + omega * trackWidth / 2.0f) / wheelRadius;
    rightMotor = (cmdVelocity - omega * trackWidth / 2.0f) / wheelRadius;
    driveMotors(leftMotor, rightMotor);
  }
}