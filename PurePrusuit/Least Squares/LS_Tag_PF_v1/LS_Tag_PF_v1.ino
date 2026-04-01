#include <PortentaUWBShield.h>
#include <math.h>
#include <Basicmicro.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// In line of sight 
// is this still being used?
// float insight_A1;
// float insight_A2;
// float insight_A3;

// Anchor Locations in cm (x,y) 
// ANCHOR LOCATIONS FROM 3/30 WRESTLING ROOM TESTING
const float Anchor1_x = 0;
const float Anchor1_y = 1273.45f;
const float Anchor2_x = 1280.16f;
const float Anchor2_y = 0;
const float Anchor3_x = 2090.01f;
const float Anchor3_y = 1125.32f;
const float Anchor4_x = 0.0f;   // <-- set 4th anchor x coord in cm
const float Anchor4_y = 0.0f;   // <-- set 4th anchor y coord in cm

// Initialize Distance Variables
int dist_1 = 0;
int dist_2 = 0;
int dist_3 = 0;
int dist_4 = 0;

// Anchor received state variables
bool anchor1_received = false;
bool anchor2_received = false;
bool anchor3_received = false;
bool anchor4_received = false;

// Previous position state variable 
double volatile currentX_global = 0.0f;
double volatile currentY_global = 0.0f;
bool prev_valid = false;

// Heading State Variable 
double Azimuth = 0.0f;
double volatile global_azimuth = 0.0f;

// Constants 
static constexpr int HALF_CIRCULAR_BUFFER_SIZE = 5;
static constexpr int CIRCULAR_BUFFER_SIZE = HALF_CIRCULAR_BUFFER_SIZE * 2;
const float MinMovement = 0.1f;
const float minMovement_sq = MinMovement * MinMovement;
static int staleCount = 0;
const int MAX_STALE = 10;
float x_circular_buffer[CIRCULAR_BUFFER_SIZE];
float y_circular_buffer[CIRCULAR_BUFFER_SIZE];
int head_index = 0;
int tail_index = CIRCULAR_BUFFER_SIZE - 1;
volatile bool inRangingHandler = false;
// Pure Pursuit Variables and Constants
// waypoint constants 
static constexpr int waypoint_radius = 100; // cm
static constexpr float look_ahead=200.0f; // cm
static constexpr float MIN_SPEED_SCALE = 0.2f; // floor at 20% of max velocity
volatile bool newPosition = false;
float delta_x; // differnce in look-ahead distance from current position  
float delta_y; // differnce in look-ahead distance from current position 
float L_d; // Look-Ahead Distance in cm 
float L_d2; // Look-Ahead Distance squared 
float K;  // Curvature Coeff (K)
float omega;  // Rotational Velocity in rad/s
const float velocity = 100.0f;  // Constant Velocity in cm/s
static constexpr float wheelRadius = 15.24; //cm 
static constexpr float trackWidth = 43.18;  // Wheel to Wheel in cm 
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

// ----------- Functions ----------- //

// Waypoint struct
struct Waypoint {
  float wp_x;
  float wp_y;
};

// Goal result struct
struct GoalResult {
  float gx;
  float gy;
  bool  found;
};

static constexpr int PATH_LENGTH = 8;

// WAYPOINTS FROM 3/30 WRESLING ROOM TESTING 
static Waypoint path[PATH_LENGTH] = {
  {1300,500},{1300,800},{950,800},{950,400},{550,400},{550,800},{200,800},{200,400}
};

float getWaypointX(int j) {
  return (j >= 0 && j < PATH_LENGTH) ? path[j].wp_x : 0.0f;
}
float getWaypointY(int j) {
  return (j >= 0 && j < PATH_LENGTH) ? path[j].wp_y : 0.0f;
}
int getPathLength() {
  return PATH_LENGTH;
}

static int pathSegIdx = 0;

void advancePathSegment() {
  if (pathSegIdx < PATH_LENGTH - 1) pathSegIdx++;
}
int getCurrentPathSegmentIndex() {
  return pathSegIdx;
}
bool PathComplete() {
  return pathSegIdx == PATH_LENGTH - 1;
}

void AdvancePathSegment() {
  if (!PathComplete()) {
    int nextWaypoint = pathSegIdx + 1;
    float deltaX = getWaypointX(nextWaypoint) - currentX_global;
    float deltaY = getWaypointY(nextWaypoint) - currentY_global;
    float distanceToNextWaypoint_sq = deltaX * deltaX + deltaY * deltaY;
    if (distanceToNextWaypoint_sq <= waypoint_radius * waypoint_radius) {
      advancePathSegment();
    }
    // could add in something to handle overshoot of waypoint radius
  }
}
//----------end waypoint handling functions----------//




//---------- start pure pursuit functions----------//

// Find intersection of lookahead circle with the path ahead.
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
    if (t2 >= 0.0f && t2 <= 1.0f) bestT = t2;
    else if (t1 >= 0.0f && t1 <= 1.0f) bestT = t1;

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
  while (a >  PI) a -= 2.0f * PI;
  while (a < -PI) a += 2.0f * PI;
  return a;
}
// Helper to convert radians to degrees
float radiansToDegrees(float a) {
  return a * (180.0f / PI);
}

void driveMotors(float leftRadPerSec, float rightRadPerSec) {
  const int32_t MAX_MOTOR_COUNTS = 70000;
  int32_t leftCounts  = constrain((int32_t)(leftRadPerSec  * RAD_TO_COUNTS), -MAX_MOTOR_COUNTS, MAX_MOTOR_COUNTS);
  int32_t rightCounts = constrain((int32_t)(rightRadPerSec * RAD_TO_COUNTS), -MAX_MOTOR_COUNTS, MAX_MOTOR_COUNTS);
  controller.SpeedAccelM1M2_2(MOTOR_ADDRESS,
                               motor_accel, (uint32_t)rightCounts,
                               motor_accel, (uint32_t)leftCounts);
}

// ----------- 4-Anchor Least Squares Trilateration ----------- //
#define N_ANCHORS 4
static const float anchor_matrix[N_ANCHORS][3] = {
  {Anchor1_x, Anchor1_y, 0.0f},
  {Anchor2_x, Anchor2_y, 0.0f},
  {Anchor3_x, Anchor3_y, 0.0f},
  {Anchor4_x, Anchor4_y, 0.0f}
};

bool trilat2D_4A(float d0, float d1, float d2, float d3, float &outX, float &outY) {
  static bool first = true;
  static float A[N_ANCHORS - 1][2];
  static float Ainv[2][2];
  static float kv[N_ANCHORS];

  float d[N_ANCHORS] = {d0, d1, d2, d3};

  if (first) {
    first = false;
    for (int i = 0; i < N_ANCHORS; i++) {
      kv[i] = anchor_matrix[i][0] * anchor_matrix[i][0]
             + anchor_matrix[i][1] * anchor_matrix[i][1];
    }
    for (int i = 1; i < N_ANCHORS; i++) {
      A[i-1][0] = anchor_matrix[i][0] - anchor_matrix[0][0];
      A[i-1][1] = anchor_matrix[i][1] - anchor_matrix[0][1];
    }
    float ATA[2][2] = {};
    for (int i = 0; i < 2; i++)
      for (int j = 0; j < 2; j++)
        for (int k = 0; k < N_ANCHORS - 1; k++)
          ATA[i][j] += A[k][i] * A[k][j];

    float det = ATA[0][0]*ATA[1][1] - ATA[1][0]*ATA[0][1];
    if (fabsf(det) < 1e-6f) {
      Serial.println("ERROR: Singular anchor matrix, check coordinates");
      return false;
    }
    det = 1.0f / det;
    Ainv[0][0] =  det * ATA[1][1];
    Ainv[0][1] = -det * ATA[0][1];
    Ainv[1][0] = -det * ATA[1][0];
    Ainv[1][1] =  det * ATA[0][0];
  }

  float b[N_ANCHORS - 1];
  for (int i = 1; i < N_ANCHORS; i++) {
    b[i-1] = d[0]*d[0] - d[i]*d[i] + kv[i] - kv[0];
  }

  float ATb[2] = {};
  for (int i = 0; i < N_ANCHORS - 1; i++) {
    ATb[0] += A[i][0] * b[i];
    ATb[1] += A[i][1] * b[i];
  }

  outX = 0.5f * (Ainv[0][0]*ATb[0] + Ainv[0][1]*ATb[1]);
  outY = 0.5f * (Ainv[1][0]*ATb[0] + Ainv[1][1]*ATb[1]);
  return true;
}

// ----------- Ranging Handler ----------- //
void rangingHandler(UWBRangingData &rangingData) {
  inRangingHandler = true;
  if (rangingData.measureType() != (uint8_t)uwb::MeasurementType::TWO_WAY) {
    inRangingHandler = false;
    return;
  }

  RangingMeasures twr = rangingData.twoWayRangingMeasure();

  for (int j = 0; j < rangingData.available(); j++) {
    if (twr[j].status != 0 || twr[j].distance == 0xFFFF) continue;

    if (twr[j].peer_addr[0] == 0x22 && twr[j].peer_addr[1] == 0x22) {
      dist_1 = twr[j].distance;
      anchor1_received = true;
    } else if (twr[j].peer_addr[0] == 0x33 && twr[j].peer_addr[1] == 0x33) {
      dist_2 = twr[j].distance;
      anchor2_received = true;
    } else if (twr[j].peer_addr[0] == 0x44 && twr[j].peer_addr[1] == 0x44) {
      dist_3 = twr[j].distance;
      anchor3_received = true;
    } else if (twr[j].peer_addr[0] == 0x55 && twr[j].peer_addr[1] == 0x55) {
      dist_4 = twr[j].distance;
      anchor4_received = true;
    }
  }

  if (!anchor1_received || !anchor2_received || !anchor3_received || !anchor4_received) {
    inRangingHandler = false;
    return;
  }

  anchor1_received = false;
  anchor2_received = false;
  anchor3_received = false;
  anchor4_received = false;

  float x, y;
  bool ok = trilat2D_4A((float)dist_1, (float)dist_2, (float)dist_3, (float)dist_4, x, y);
  if (!ok) {
    inRangingHandler = false;
    return;
  }

  if (!prev_valid) {
    for (int i = 0; i < CIRCULAR_BUFFER_SIZE; i++) {
      x_circular_buffer[i] = x;
      y_circular_buffer[i] = y;
    }
    prev_valid = true;
    inRangingHandler = false;
    return;
  }

  // Circular Buffer Advance
  head_index++;
  if (head_index == CIRCULAR_BUFFER_SIZE) head_index = 0;
  tail_index++;
  if (tail_index == CIRCULAR_BUFFER_SIZE) tail_index = 0;
  x_circular_buffer[head_index] = x;
  y_circular_buffer[head_index] = y;

  float currentX = 0.0f;
  float currentY = 0.0f;
  float prevX    = 0.0f;
  float prevY    = 0.0f;

  // Weighted Average
  float weights[HALF_CIRCULAR_BUFFER_SIZE];
  weights[0] = 0.3f;
  weights[1] = 0.25f;
  weights[2] = 0.2f;
  weights[3] = 0.15f;
  weights[4] = 0.1f;

  int index = head_index;
  for (int i = 0; i < HALF_CIRCULAR_BUFFER_SIZE; i++) {
    if (index < 0) index += CIRCULAR_BUFFER_SIZE;
    currentX += weights[i] * x_circular_buffer[index];
    currentY += weights[i] * y_circular_buffer[index];
    index--;
  }

  index = tail_index;
  for (int i = 0; i < HALF_CIRCULAR_BUFFER_SIZE; i++) {
    if (index >= CIRCULAR_BUFFER_SIZE) index -= CIRCULAR_BUFFER_SIZE;
    prevX += weights[i] * x_circular_buffer[index];
    prevY += weights[i] * y_circular_buffer[index];
    index++;
  }

  currentX_global = currentX;
  currentY_global = currentY;

  // Heading Calculation
  float dx = currentX - prevX;
  float dy = currentY - prevY;
  float dist_sq = dx * dx + dy * dy;

  if (dist_sq >= minMovement_sq) {
    Azimuth = atan2f(dy, dx);
    global_azimuth = Azimuth;
    staleCount = 0;
  } else {
    staleCount++;
    if (staleCount >= MAX_STALE) {
      float minDrive = 0.5f * velocity / wheelRadius;
      driveMotors(-minDrive, -minDrive);
    }
  }

  newPosition = true;
  inRangingHandler = false;
}

void setup() {
  Serial.begin(115200);

#if defined(ARDUINO_PORTENTA_C33)
  pinMode(LEDR, OUTPUT);
  digitalWrite(LEDR, LOW);
#endif

  uint8_t devAddr[]      = {0x11, 0x11};
  uint8_t destination1[] = {0x22, 0x22};
  uint8_t destination2[] = {0x33, 0x33};
  uint8_t destination3[] = {0x44, 0x44};
  uint8_t destination4[] = {0x55, 0x55};

  UWBMacAddress srcAddr(UWBMacAddress::Size::SHORT, devAddr);
  UWBMacAddress dstAddr1(UWBMacAddress::Size::SHORT, destination1);
  UWBMacAddress dstAddr2(UWBMacAddress::Size::SHORT, destination2);
  UWBMacAddress dstAddr3(UWBMacAddress::Size::SHORT, destination3);
  UWBMacAddress dstAddr4(UWBMacAddress::Size::SHORT, destination4);

  UWBMacAddressList dest(UWBMacAddress::Size::SHORT);
  dest.add(dstAddr1);
  dest.add(dstAddr2);
  dest.add(dstAddr3);
  dest.add(dstAddr4);

  UWB.registerRangingCallback(rangingHandler);
  UWB.begin();
  Serial.println("Starting UWB ...");

  while (UWB.state() != 0) delay(10);

  Serial.println("Starting multicast session ...");
  UWBRangingOneToMany myController(0x11223344, srcAddr, dest);
  UWBSessionManager.addSession(myController);
  myController.init();
  myController.start();

  controller.begin(38400);
  delay(100);
  controller.SetM1VelocityPID(MOTOR_ADDRESS, 1.79279, 0.27940, 0.00000, 70620);
  controller.SetM2VelocityPID(MOTOR_ADDRESS, 1.74675, 0.26201, 0.00000, 69630);
}

void loop() {
#if defined(ARDUINO_PORTENTA_C33)
  digitalWrite(LEDR, !digitalRead(LEDR));
#endif
  delay(10);
  while (inRangingHandler || !newPosition) {
    delay(10);
  }
  newPosition = false;
  AdvancePathSegment();
  if (PathComplete()) {
    driveMotors(0, 0);
    Serial.println("Path Complete");
    inRangingHandler = false;
    return;
  }

  GoalResult goal = findLookaheadGoal();

  delta_x = goal.gx - currentX_global;
  delta_y = goal.gy - currentY_global;

  float angleToGoal = atan2f(delta_y, delta_x);
  Serial.println(goal.gx);
  Serial.println(goal.gy);
  float DesiredHeading = radiansToDegrees(angleToGoal);
  Serial.println(DesiredHeading);
  float azimuth_deg = radiansToDegrees(global_azimuth);
  Serial.println(azimuth_deg);
  Serial.println(currentX_global);
  Serial.println(currentY_global);

  L_d2 = delta_x * delta_x + delta_y * delta_y;
  L_d  = sqrtf(L_d2);

  float alpha = wrapAnglePi(angleToGoal - global_azimuth);
  float cmdVelocity = velocity;

  if (L_d < 1.0f) {
    K = 0.0f;
    driveMotors(0, 0);
  } else {
    K = 2.0f * sinf(alpha) / L_d;
    omega      = K * cmdVelocity;
    leftMotor  = (cmdVelocity - omega * trackWidth / 2.0f) / wheelRadius;
    rightMotor = (cmdVelocity + omega * trackWidth / 2.0f) / wheelRadius;
    driveMotors(leftMotor, rightMotor);
  }
}