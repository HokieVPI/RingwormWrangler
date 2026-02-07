#include <PortentaUWBShield.h>
#include <math.h>


/**
 * This Script claculates the position of an Arduino Protenta Tag based 
 * on the distances to three anchors. Using multicast to range with 3+ anchors
 * UWB Ranging Controller (one-to-many)*
 * It expects multiple counterparts setup as Responders/Controlees*
 */

// Anchor Locations in Centimeters (x,y) z=0 
const float Anchor1_x=0;// cm 
const float Anchor1_y=0; // cm 
const float Anchor2_x=0; // cm 
const float Anchor2_y=213; // cm 
const float Anchor3_x=182; // cm 
const float Anchor3_y=0; // cm 

bool anchor1_received = false;
bool anchor2_received = false;
bool anchor3_received = false;
//previous position state variable 
float prevX = 0.0f;// cm 
float prevY = 0.0f;// cm 
float currentX = 0.0f;// cm 
float currentY = 0.0f;// cm 
bool prev_valid = false;

// Heading State Variable 
float Azimuth = 0.0f; // rad


// constants 
const float MinMovement = 5.0f; // cm 
const float minMovement_sq=MinMovement*MinMovement; // minimum movement squared
float x_circular_buffer[5];
float y_circular_buffer[5];
static constexpr int CIRCULAR_BUFFER_SIZE = 5;
int head_index = 0;
int tail_index = CIRCULAR_BUFFER_SIZE-1;
bool inRangingHandler = false;

// Pure Prusuit var
static constexpr int LOOK_AHEAD = 100; // cm
static constexpr int INTERVAL = 100; // ms
static constexpr int STOP_POINT = 10; // cm
static constexpr int PATH_LEN = 4;
static float pathX[PATH_LEN] = {0,100,200,300}; //waypoints in cm 
static float pathY[PATH_LEN] = {0,100,200,300}; // waypoints in cm 
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

static constexpr int INTERPOLATION_STEP = PurePursuit::DEFAULT_INTERPOLATION_STEP;
PurePursuit pp(&currentX, &currentY, &Azimuth, LOOK_AHEAD, INTERVAL, STOP_POINT);




// handler for ranging notifications
void rangingHandler(UWBRangingData &rangingData) {
  inRangingHandler = true;
  if(rangingData.measureType()==(uint8_t)uwb::MeasurementType::TWO_WAY)
  {

    RangingMeasures twr=rangingData.twoWayRangingMeasure();

  int dist_1 = 1;
  int dist_2 = 1;
  int dist_3 = 1; 

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
  return; // wait for all data
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

  currentX = x;
  currentY = y;
  Serial.print("Position: ");
  Serial.print(x);
  Serial.print(", ");
  Serial.println(y);
  //
  prevX = x_circular_buffer[tail_index];
  prevY = y_circular_buffer[tail_index];

    float dx = x - prevX;
    float dy = y - prevY;
    float dist_sq = dx*dx + dy*dy;
    if (dist_sq >= minMovement_sq) {
      Azimuth = atan2f(dy, dx);
                Serial.println(dist_sq);

     }else{

 
  Serial.print("Azimuth invalid--Min Movement");
          Serial.println(dist_sq);
              Serial.print("Heading: ");
    Serial.println(Azimuth);

  return;
       }

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

  
  //convert to degrees 
  Azimuth=Azimuth*(180.0/PI);
  // print the heading
    Serial.print("Heading: ");
    Serial.println(Azimuth);

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

  while(inRangingHandler) {
    delay(10);
  }

  [goal_x, goal_y] = pp.nextGoalPoint();
    x_i=goal_x-currentX; // differnce from goal point to current position 
    y_i=goal_y-currentY; // differnce from goal point to current position
// Find Look-ahead Distance 
L_d2=x_i*x_i+y_i*y_i;  // Look-ahead Distance Squared 
L_d=sqrt(L_d2); // Look-ahead distance
// Compute the Curvature Coeff (K)
K=2*sinf(Azimuth)/L_d; 
omega=K*Velocity; 

// // Find Motor Velocities 
// leftMotor=(velocity+omega*trackWidth/2)/wheelRadius; 
// rightMotor=(velocity+omega*trackWidth/2)/wheelRadius; 

  delay(1000);
}