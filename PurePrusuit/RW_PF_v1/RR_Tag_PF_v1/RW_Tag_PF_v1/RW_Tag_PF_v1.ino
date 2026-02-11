#include <PortentaUWBShield.h>
#include <math.h>
#include <PurePursuit.h>


/**
 * pure pursuit path following algorithm for a single tag
 */

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

// Pure Prusuit var
static constexpr int LOOK_AHEAD = 1000; // cm
static constexpr int INTERVAL = 1000; // ms
static constexpr int STOP_POINT = 50; // cm
static constexpr int PATH_LEN = 4;
static float pathX[PATH_LEN] = {200,792,792,792}; //waypoints in cm 
static float pathY[PATH_LEN] = {200,1321,1864,2441 }; // waypoints in cm 
float x_i; // differnce in look-ahead distance from current position  
float y_i; // differnce in look-ahead distance from current position 
float L_d; // Look-Ahead Distance in cm 
float L_d2; // Look-Ahead Distance squared 
float K; // Curvature Coeff (K)
float omega; // Rotational Velocity in rad/s
const float velocity = 0.5f;  // Constant Velocity in m/s
static constexpr int wheelRadius = 15;  //cm 
static constexpr int trackWidth =86;  // Wheel to Wheel in cm 
float leftMotor; 
float rightMotor; 

bool new_goal = false;

static int INTERPOLATION_STEP =250; //mm;
PurePursuit pp(&currentX_global, &currentY_global , &Azimuth, LOOK_AHEAD, INTERVAL, STOP_POINT);




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

  Azimuth=global_azimuth *(180.0/PI);
  // Serial.print("Heading: ");
  // Serial.println(Azimuth);

  new_goal = true;
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

// Set up the Pure Pursuit path
pp.setPath(pathX, pathY, PATH_LEN);
pp.setInterpolationStep(INTERPOLATION_STEP); 
pp.start();
}

void loop() {
  #if defined(ARDUINO_PORTENTA_C33)
  /* Only the Portenta C33 has an RGB LED. */
  digitalWrite(LEDR, !digitalRead(LEDR));
#endif

  while(inRangingHandler || !new_goal) {
    
    delay(10);
  }
  new_goal = false;
  pp.computeNextGoalPoint();

  float goal_x = pp.getGoalX();
  float goal_y = pp.getGoalY();


Serial.println(goal_x);
Serial.println(goal_y);

    x_i=goal_x-currentX_global; // differnce from goal point to current position 
    y_i=goal_y-currentY_global; // differnce from goal point to current position
// Find Look-ahead Distance 
float DesiredHeading=0.0f;
DesiredHeading=atan2f(y_i, x_i);
DesiredHeading=DesiredHeading*(180.0/PI);
// Serial.print("Desired Heading: ");
// Serial.println(DesiredHeading);
  //  Serial.print("Global Heading: ");
  //   Serial.println(global_azimuth);
  Serial.println(currentX_global);
  Serial.println(currentY_global);
 
L_d2=x_i*x_i+y_i*y_i;  // Look-ahead Distance Squared 
L_d=sqrt(L_d2); // Look-ahead distance

// Compute the Curvature Coeff (K)
K=2*sinf(global_azimuth)/L_d; 
// Serial.print("Curvature Coeff: ");
// Serial.println(K);
// omega=K*velocity; 

// // Find Motor Velocities 
// leftMotor=(velocity+omega*trackWidth/2)/wheelRadius; 
// rightMotor=(velocity+omega*trackWidth/2)/wheelRadius; 
// Serial.println(leftMotor);
// Serial.println(rightMotor);
}