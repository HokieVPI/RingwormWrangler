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
// const int S1Pin = 13; // S1 used for roboclaw 
// const int S2Pin = 14; // S2



// orange, blue, white, green
// 0, 1, 2, 4


/**
 * pure pursuit path following algorithm for a single tag
 working on implementing custom pure pursuit path following algorithm

 used cursor--- need to verify code 
 **/

//------####------ Global Variables ------####------//

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

// Heading State Variable 
double volatile global_azimuth = 0.0f; // rad

// pure pursuit variables & constants 
// waypoint constant
// for wrestling room
static constexpr int waypoint_radius = 150; // cm
static constexpr int final_waypoint_radius = 200; // cm (larger radius only for final waypoint)
static constexpr float look_ahead=170.0f; // cm

// for lab space
// static constexpr int waypoint_radius = 50; // cm
// static constexpr int final_waypoint_radius = 100; // cm (larger radius only for final waypoint)
// static constexpr float look_ahead=100.0f; // cm

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

// Set to 1 for Serial Monitor EKF diagnostics ([EKF] lines). Set to 0 to disable.
#ifndef DEBUG_EKF_SERIAL
#define DEBUG_EKF_SERIAL 1
#endif

static constexpr float ENCODER_CPR   = 24293.0f;
static constexpr float RAD_TO_COUNTS = ENCODER_CPR / (2.0f * PI);
uint32_t motor_accel = 10000;
static constexpr uint32_t ControlLoop_period_ms = 100;  // 10 Hz control loop
static constexpr float QX_BASE = 80.0f;              // cm^2 / step
static constexpr float QY_BASE = 80.0f;              // cm^2 / step
static constexpr float QTH_BASE = 0.0600f;          // rad^2 / step
static constexpr float RX = 100.0f;                 // cm^2
static constexpr float RY = 100.0f;                 // cm^2
static constexpr float NIS_GATE = 25.0f;
static constexpr uint32_t UWB_stale_ms = 1500;
static constexpr uint32_t Q_SCALE_15_MS = 6000;
static constexpr uint32_t Q_SCALE_20_MS = 15000;

struct EKFState {
  float x;
  float y;
  float theta;
};

struct UWBMeasurement {
  float x;
  float y;
  uint32_t timestampMs;
  bool fresh;
  bool valid;
};

static EKFState ekf = {0.0f, 0.0f, 0.0f};
static float P[3][3] = {
  {400.0f, 0.0f, 0.0f},
  {0.0f, 400.0f, 0.0f},
  {0.0f, 0.0f, 0.25f}
};
static volatile UWBMeasurement latestUwb = {0.0f, 0.0f, 0, false, false};
static bool ekfInitialized = false;
static uint32_t lastAcceptedUwbMs = 0;
static uint32_t lastControlTickMs = 0;
static bool encoderInitialized = false;
static uint32_t prevEncM1 = 0;
static uint32_t prevEncM2 = 0;

Basicmicro controller(&controllerSerial, LIBRARY_READ_TIMEOUT);

// Linear Actuator Motor Controller: Retract = pin 6, Extend = pin 7

const int RetractPin = 6;
const int ExtentPin = 7;
const int PumpPin = 1; // orange
const int SolenoidPin = 2; // green

//------####------ Functions ------####------//

//------####------ Waypoint + Cleaning Helpers ------####------//

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

static constexpr int PATH_LENGTH = 32;
static Waypoint path[PATH_LENGTH] = {
  {250.0, 500.0},
  {250.0, 2400.5},
  {333.0, 2400.5},
  {416.0, 2400.5},
  {416.0, 350.0},
  {499.0, 250.0},
  {582.0, 250.0},
  {582.0, 2400.5},
  {665.0, 2400.5},
  {748.0, 2400.5},
  {748.0, 250.0},
  {831.0, 250.0},
  {914.0, 250.0},
  {914.0, 2400.5},
  {997.0, 2400.5},
  {1080.0, 2400.5},
  {1080.0, 250.0},
  {1163.0, 250.0},
  {1246.0, 250.0},
  {1246.0, 1890.9},
  {1329.0, 1890.9},
  {1412.0, 1890.9},
  {1412.0, 250.0},
  {1495.0, 250.0},
  {1578.0, 250.0},
  {1578.0, 1341.4},
  {1661.0, 1341.4},
  {1744.0, 1341.4},
  {1744.0, 250.0},
  {997.0, 250.0},
  {250.0, 250.0},
  {250.0, 500.0}
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
//-- Function: getWaypointX --//
// Return waypoint X in cm for a valid index.
float getWaypointX(int j){
  return (j>=0 && j<PATH_LENGTH) ? path[j].wp_x : 0.0f;
}
//-- Function: getWaypointY --//
// Return waypoint Y in cm for a valid index.
float getWaypointY(int j){
  return (j>=0 && j<PATH_LENGTH) ? path[j].wp_y : 0.0f;
}

//-- Function: getPathLength --//
// Return total number of waypoints in the active path.
int getPathLength(){
  return PATH_LENGTH;
}
// establish path state 
static int pathSegIdx = 0; // current path segment index

//-- Function: advancePathSegment --//
// Move to the next path segment index (bounded to final segment).
void advancePathSegment(){
  if(pathSegIdx < PATH_LENGTH - 1){
    pathSegIdx++;
  }

}
//-- Function: getCurrentPathSegmentIndex --//
// Return current segment index used by path tracking.
int getCurrentPathSegmentIndex(){
  return pathSegIdx;
}
//-- Function: PathComplete --//
// True when robot reached the final segment index.
bool PathComplete(){
  return
   pathSegIdx == PATH_LENGTH - 1;
}


// Pump on + solenoid off pressurizes bladder while spray-active (no timers).
// After PathComplete(), CleaningStage is incremented before SprayActive(); stage 1 is the first cleaning phase.
//-- Function: sprayOutputsActive --//
// Enable spray outputs only during the spraying stage.
static bool sprayOutputsActive() {
  return CleaningStage == 0;
}

//-- Function: applySprayOutputs --//
// Toggle pump/solenoid outputs according to current cleaning stage.
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

//-- Function: SprayActive --//
// Wrapper to apply current spray output policy.
void SprayActive() {
  applySprayOutputs();
}

//-- Function: MopActive --//
// Move linear actuator according to cleaning stage.
void MopActive() {
  if (CleaningStage == 1) {
    digitalWrite(ExtentPin, HIGH);
    digitalWrite(RetractPin, LOW);
    delay(ActuatorDuration);
    digitalWrite(ExtentPin, LOW);

  } else if (CleaningStage == 2) {
    // digitalWrite(ExtentPin, LOW);
    // digitalWrite(RetractPin, HIGH);
    // delay(ActuatorDuration);
    // digitalWrite(RetractPin, LOW);
    // // this is temporary fix for bad port 6 connection for retracting
    digitalWrite(ExtentPin, HIGH);
    digitalWrite(RetractPin, LOW);
    delay(ActuatorDuration);
    digitalWrite(ExtentPin, LOW);

  } else {
    digitalWrite(ExtentPin, LOW);
    digitalWrite(RetractPin, LOW);
  }
}


// advance when the tag is within the waypoint radius of the next waypoint
//-- Function: AdvancePathSegment --//
// Advance only when robot is inside the acceptance radius of next waypoint.
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
//------####------ Pure Pursuit Helpers ------####------//


// Find intersection of lookahead circle with the path ahead.
// Uses: currentX_global, currentY_global, look_ahead, path[], pathSegIdx, PATH_LENGTH
//-- Function: findLookaheadGoal --//
// Compute a pursuit goal on the current/future path segment.
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

//------####------ EKF + UWB Helpers ------####------//
// Helper to wrap angle to [-PI, PI]
//-- Function: wrapAnglePi --//
// Normalize any angle to [-PI, PI] for stable steering math.
static float wrapAnglePi(float a) {
  while (a > PI)  a -= 2.0f * PI;
  while (a < -PI) a += 2.0f * PI;
  return a;
}
// Helper to convert radians to degrees
//-- Function: radiansToDegrees --//
// Convert angle from radians to degrees (debug/telemetry use).
float radiansToDegrees(float a) {
  return a * (180.0/PI);
}

//-- Function: setCovariance --//
// Reset EKF covariance matrix P to a diagonal initializer.
static void setCovariance(float pxx, float pyy, float ptt) {
  P[0][0] = pxx; P[0][1] = 0.0f; P[0][2] = 0.0f;
  P[1][0] = 0.0f; P[1][1] = pyy; P[1][2] = 0.0f;
  P[2][0] = 0.0f; P[2][1] = 0.0f; P[2][2] = ptt;
}

//-- Function: initialPathHeading --//
// Use first path segment direction as initial EKF heading.
static float initialPathHeading() {
  if (PATH_LENGTH < 2) return 0.0f;
  float dx = path[1].wp_x - path[0].wp_x;
  float dy = path[1].wp_y - path[0].wp_y;
  return atan2f(dy, dx);
}

//-- Function: isUwbStale --//
// Mark UWB measurements stale after timeout threshold.
static bool isUwbStale(uint32_t nowMs, uint32_t measMs) {
  return (nowMs - measMs) > UWB_stale_ms;
}

//-- Function: processQScale --//
// Increase process noise as time since last accepted UWB grows.
static float processQScale(uint32_t nowMs) {
  uint32_t sinceAccepted = nowMs - lastAcceptedUwbMs;
  if (sinceAccepted > Q_SCALE_20_MS) return 2.0f;
  if (sinceAccepted > Q_SCALE_15_MS) return 1.5f;
  return 1.0f;
}

//-- Function: predictFromEncoders --//
// EKF predict step from wheel encoder odometry and covariance propagation.
static void predictFromEncoders(uint32_t nowMs) {
  uint8_t s1 = 0, s2 = 0;
  bool validM1 = false, validM2 = false;
  uint32_t encM1 = controller.ReadEncM1(MOTOR_ADDRESS, &s1, &validM1); // right
  uint32_t encM2 = controller.ReadEncM2(MOTOR_ADDRESS, &s2, &validM2); // left
  if (!validM1 || !validM2) return;

  // First valid read is used only to seed previous counts.
  if (!encoderInitialized) {
    prevEncM1 = encM1;
    prevEncM2 = encM2;
    encoderInitialized = true;
    return;
  }

  // Signed deltas preserve direction and handle count wrap naturally.
  int32_t dCountR = (int32_t)(encM1 - prevEncM1); // M1 is right
  int32_t dCountL = (int32_t)(encM2 - prevEncM2); // M2 is left
  prevEncM1 = encM1;
  prevEncM2 = encM2;

  // Convert encoder counts -> wheel angle -> wheel travel.
  float dPhiR = (2.0f * PI) * ((float)dCountR / ENCODER_CPR);
  float dPhiL = (2.0f * PI) * ((float)dCountL / ENCODER_CPR);
  float dR = wheelRadius * dPhiR;
  float dL = wheelRadius * dPhiL;
  float dS = 0.5f * (dR + dL);
  float dTheta = (dR - dL) / trackWidth;
  float midHeading = ekf.theta + 0.5f * dTheta;

  ekf.x += dS * cosf(midHeading);
  ekf.y += dS * sinf(midHeading);
  ekf.theta = wrapAnglePi(ekf.theta + dTheta);

  // Linearized motion Jacobian for covariance propagation.
  float F[3][3] = {
    {1.0f, 0.0f, -dS * sinf(midHeading)},
    {0.0f, 1.0f,  dS * cosf(midHeading)},
    {0.0f, 0.0f, 1.0f}
  };
  float FP[3][3] = {{0}};
  float FPFt[3][3] = {{0}};
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      for (int k = 0; k < 3; k++) {
        FP[r][c] += F[r][k] * P[k][c];
      }
    }
  }
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      for (int k = 0; k < 3; k++) {
        FPFt[r][c] += FP[r][k] * F[c][k];
      }
    }
  }
  // Inflate process noise when UWB has been missing for longer periods.
  float qScale = processQScale(nowMs);
  float Q[3] = {QX_BASE * qScale, QY_BASE * qScale, QTH_BASE * qScale};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      P[i][j] = FPFt[i][j];
    }
  }
  P[0][0] += Q[0];
  P[1][1] += Q[1];
  P[2][2] += Q[2];

#if DEBUG_EKF_SERIAL
  Serial.print("[EKF] pred dS_cm=");
  Serial.print(dS, 3);
  Serial.print(" dTh_deg=");
  Serial.print(dTheta * 180.0f / PI, 3);
  Serial.print(" qScale=");
  Serial.print(qScale, 2);
  Serial.print(" | x=");
  Serial.print(ekf.x, 2);
  Serial.print(" y=");
  Serial.print(ekf.y, 2);
  Serial.print(" th_deg=");
  Serial.print(ekf.theta * 180.0f / PI, 2);
  Serial.print(" Pdiag=");
  Serial.print(P[0][0], 1);
  Serial.print(",");
  Serial.print(P[1][1], 1);
  Serial.print(",");
  Serial.println(P[2][2], 4);
#endif
}

//-- Function: tryUpdateFromUWB --//
// EKF correction step using latest fresh UWB x/y measurement.
static bool tryUpdateFromUWB(uint32_t nowMs) {
  // Snapshot latest measurement atomically, then mark it consumed.
  noInterrupts();
  UWBMeasurement m;
  m.x = latestUwb.x;
  m.y = latestUwb.y;
  m.timestampMs = latestUwb.timestampMs;
  m.fresh = latestUwb.fresh;
  m.valid = latestUwb.valid;
  if (m.fresh) {
    latestUwb.fresh = false;
  }
  interrupts();
  if (!m.fresh || !m.valid || isUwbStale(nowMs, m.timestampMs)) {
#if DEBUG_EKF_SERIAL
    Serial.print("[EKF] UWB skip: ");
    if (!m.fresh) {
      Serial.println("not fresh");
    } else if (!m.valid) {
      Serial.println("invalid");
    } else {
      Serial.print("stale age_ms=");
      Serial.println((unsigned long)(nowMs - m.timestampMs));
    }
#endif
    return false;
  }

  // Innovation: how far measurement is from predicted state.
  float innov[2] = {m.x - ekf.x, m.y - ekf.y};
  float S00 = P[0][0] + RX;
  float S01 = P[0][1];
  float S10 = P[1][0];
  float S11 = P[1][1] + RY;
  float detS = S00 * S11 - S01 * S10;
  if (fabsf(detS) < 1e-6f) {
#if DEBUG_EKF_SERIAL
    Serial.println("[EKF] UWB skip: detS near zero");
#endif
    return false;
  }

  float iS00 =  S11 / detS;
  float iS01 = -S01 / detS;
  float iS10 = -S10 / detS;
  float iS11 =  S00 / detS;
  // NIS gating rejects large outliers (e.g., bad UWB geometry/NLOS).
  float nis = innov[0] * (iS00 * innov[0] + iS01 * innov[1]) +
              innov[1] * (iS10 * innov[0] + iS11 * innov[1]);
  if (nis > NIS_GATE) {
#if DEBUG_EKF_SERIAL
    Serial.print("[EKF] UWB skip: NIS=");
    Serial.print(nis, 2);
    Serial.print(" > gate ");
    Serial.println(NIS_GATE, 2);
#endif
    return false;
  }

  float K[3][2];
  K[0][0] = P[0][0] * iS00 + P[0][1] * iS10;
  K[0][1] = P[0][0] * iS01 + P[0][1] * iS11;
  K[1][0] = P[1][0] * iS00 + P[1][1] * iS10;
  K[1][1] = P[1][0] * iS01 + P[1][1] * iS11;
  K[2][0] = P[2][0] * iS00 + P[2][1] * iS10;
  K[2][1] = P[2][0] * iS01 + P[2][1] * iS11;

  float prevTheta = ekf.theta;
  ekf.x += K[0][0] * innov[0] + K[0][1] * innov[1];
  ekf.y += K[1][0] * innov[0] + K[1][1] * innov[1];
  ekf.theta += K[2][0] * innov[0] + K[2][1] * innov[1];
  ekf.theta = wrapAnglePi(ekf.theta);
  float dThetaCorr = wrapAnglePi(ekf.theta - prevTheta);

  float KH[3][3] = {
    {K[0][0], K[0][1], 0.0f},
    {K[1][0], K[1][1], 0.0f},
    {K[2][0], K[2][1], 0.0f}
  };
  float IminusKH[3][3] = {
    {1.0f - KH[0][0], -KH[0][1], 0.0f},
    {-KH[1][0], 1.0f - KH[1][1], 0.0f},
    {-KH[2][0], -KH[2][1], 1.0f}
  };
  float newP[3][3] = {{0}};
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      for (int k = 0; k < 3; k++) {
        newP[r][c] += IminusKH[r][k] * P[k][c];
      }
    }
  }
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      P[r][c] = newP[r][c];
    }
  }
  lastAcceptedUwbMs = nowMs;
#if DEBUG_EKF_SERIAL
  Serial.print("[EKF] UWB upd innov_cm=(");
  Serial.print(innov[0], 2);
  Serial.print(",");
  Serial.print(innov[1], 2);
  Serial.print(") NIS=");
  Serial.print(nis, 2);
  Serial.print(" dTh_corr_deg=");
  Serial.print(dThetaCorr * 180.0f / PI, 3);
  Serial.print(" | x=");
  Serial.print(ekf.x, 2);
  Serial.print(" y=");
  Serial.print(ekf.y, 2);
  Serial.print(" th_deg=");
  Serial.println(ekf.theta * 180.0f / PI, 2);
#endif
  return true;
}

//-- Function: tryInitializeFromUwb --//
// Bootstrap EKF state from first valid fresh UWB measurement.
static bool tryInitializeFromUwb(uint32_t nowMs) {
  // Same atomic snapshot pattern used by normal EKF updates.
  noInterrupts();
  UWBMeasurement m;
  m.x = latestUwb.x;
  m.y = latestUwb.y;
  m.timestampMs = latestUwb.timestampMs;
  m.fresh = latestUwb.fresh;
  m.valid = latestUwb.valid;
  if (m.fresh) {
    latestUwb.fresh = false;
  }
  interrupts();
  if (!m.fresh || !m.valid || isUwbStale(nowMs, m.timestampMs)) {
#if DEBUG_EKF_SERIAL
    Serial.print("[EKF] init skip: ");
    if (!m.fresh) {
      Serial.println("not fresh");
    } else if (!m.valid) {
      Serial.println("invalid");
    } else {
      Serial.print("stale age_ms=");
      Serial.println((unsigned long)(nowMs - m.timestampMs));
    }
#endif
    return false;
  }
  ekf.x = m.x;
  ekf.y = m.y;
  // Seed heading from the first path segment so steering starts coherently.
  ekf.theta = initialPathHeading();
  setCovariance(400.0f, 400.0f, 0.25f);
  ekfInitialized = true;
  lastAcceptedUwbMs = nowMs;
#if DEBUG_EKF_SERIAL
  Serial.print("[EKF] init x=");
  Serial.print(ekf.x, 2);
  Serial.print(" y=");
  Serial.print(ekf.y, 2);
  Serial.print(" th_deg=");
  Serial.print(ekf.theta * 180.0f / PI, 2);
  Serial.print(" Pdiag=400,400,0.25");
  Serial.println();
#endif
  return true;
}

//-- Function: driveMotors --//
// Convert wheel angular rates to RoboClaw speed commands.
void driveMotors(float leftRadPerSec, float rightRadPerSec) {
  const int32_t MAX_MOTOR_COUNTS = 70000;
  int32_t leftCounts  = constrain((int32_t)(leftRadPerSec  * RAD_TO_COUNTS), -MAX_MOTOR_COUNTS, MAX_MOTOR_COUNTS);
  int32_t rightCounts = constrain((int32_t)(rightRadPerSec * RAD_TO_COUNTS), -MAX_MOTOR_COUNTS, MAX_MOTOR_COUNTS);
  controller.SpeedAccelM1M2_2(MOTOR_ADDRESS,
                               motor_accel, rightCounts,
                               motor_accel, leftCounts);
}

// Trilateration using Cramer's rule on any 3 anchors (indices into anchorX/Y arrays).
// Returns true on success, writes result into *outX, *outY.
//-- Function: trilaterate --//
// Solve tag position from three anchor distances.
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

//------####------ UWB Callback ------####------//
// handler for ranging notifications
//-- Function: rangingHandler --//
// Parse UWB ranges, trilaterate, and publish the latest measurement snapshot.
void rangingHandler(UWBRangingData &rangingData) {
  if(rangingData.measureType()==(uint8_t)uwb::MeasurementType::TWO_WAY)
  {

    RangingMeasures twr=rangingData.twoWayRangingMeasure();


  // Parse all peers in this callback and keep only valid range values.
  for (int j = 0; j < rangingData.available(); j++) {
    // skip invalid measurements
    if (twr[j].status != 0 || twr[j].distance == 0xFFFF) {
      continue;
    }
  
    // classify by short MAC (first two bytes)
    if (twr[j].peer_addr[0] == 0x22 && twr[j].peer_addr[1] == 0x22) {
      dist_1 = twr[j].distance;
      anchor1_received = true;
      // Serial.println(dist_1);
    } else if (twr[j].peer_addr[0] == 0x33 && twr[j].peer_addr[1] == 0x33) {
      dist_2 = twr[j].distance;
      anchor2_received = true;
      // Serial.println(dist_2);
      } else if (twr[j].peer_addr[0] == 0x44 && twr[j].peer_addr[1] == 0x44) {
      dist_3 = twr[j].distance;
      anchor3_received = true;
      // Serial.println(dist_3);
    } else if (twr[j].peer_addr[0] == 0x55 && twr[j].peer_addr[1] == 0x55) {
      dist_4 = twr[j].distance;
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

  // We need at least 3 anchors for 2D trilateration.
  int nValid = anchor1_received + anchor2_received
             + anchor3_received + anchor4_received;

  anchor1_received = false;
  anchor2_received = false;
  anchor3_received = false;
  anchor4_received = false;

  if (nValid < 3) {
    return;
  }

  float x = 0.0f, y = 0.0f;
  bool solved = false;
  // Try preferred anchor combinations until one solves.
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
    return;
  }

  // Publish one fresh measurement atomically for loop() consumption.
  noInterrupts();
  latestUwb.x = x;
  latestUwb.y = y;
  latestUwb.timestampMs = millis();
  latestUwb.fresh = true;
  latestUwb.valid = true;
  interrupts();
  }
}

//------####------ Setup ------####------//
//-- Function: setup --//
// Configure hardware, UWB session, motor controller, and initial state.
void setup() {

  // Start serial before any diagnostics and board-level init.
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
 

  // Register callback before UWB session start so early packets are captured.
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
  
  
  // Initialize RoboClaw UART and velocity PID gains.
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
  digitalWrite(SolenoidPin, HIGH);

  digitalWrite(ExtentPin, LOW); // to raise the mop 
  digitalWrite(RetractPin, HIGH);
  delay(ActuatorDuration);
  digitalWrite(RetractPin, LOW);

  // Initialize EKF/control timers to current time baseline.
  lastControlTickMs = millis();
  lastAcceptedUwbMs = lastControlTickMs;
}

//------####------ Loop ------####------//
//-- Function: loop --//
// Run fixed-rate EKF + pure pursuit control and cleaning-stage transitions.
void loop() {
  // Fixed-period control loop: run once every ControlLoop_period_ms.
  uint32_t now = millis();
  if ((now - lastControlTickMs) < ControlLoop_period_ms) {
    return;
  }
  lastControlTickMs = now;  // drop missed ticks to avoid burst catch-up after delays

#if defined(ARDUINO_PORTENTA_C33)
  /* Only the Portenta C33 has an RGB LED. */
  digitalWrite(LEDR, !digitalRead(LEDR));
#endif

  // Hold motors still until first valid UWB-based EKF initialization.
  if (!ekfInitialized) {
    if (!tryInitializeFromUwb(now)) {
      driveMotors(0.0f, 0.0f);
      return;
    }
  }

  // Predict from encoders every tick; correct with UWB when available.
  predictFromEncoders(now);
  tryUpdateFromUWB(now);

  // Publish EKF state for navigation/pure-pursuit consumers.
  currentX_global = ekf.x;
  currentY_global = ekf.y;
  global_azimuth = ekf.theta;

  AdvancePathSegment(); // check if we reached the next waypoint
  // applySprayOutputs();  // hold pump/solenoid state for whole time CleaningStage == 1
  // digitalWrite(RoboClawFusePin, HIGH);
  // Handle cleaning-stage transitions when path reaches final segment.
  if (PathComplete()) {
    // Advance cleaning stage
    CleaningStage = CleaningStage + 1;
    // does pass again
    controller.SpeedAccelM1M2(MOTOR_ADDRESS, motor_accel, 0, 0);
    delay(3000);
    pathSegIdx = 0;
    
    SprayActive();
    MopActive();
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
    }
  }

  // Compute lookahead target and steering geometry in robot frame.
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

  // float speedScale = 0.5f;
  // if (fabsf(alpha) > PI / 4.0f) {
  //   speedScale = (PI - fabsf(alpha)) / PI;
  // }
   float cmdVelocity = velocity;

  // Stop at near-zero lookahead distance to avoid noisy steering spikes.
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