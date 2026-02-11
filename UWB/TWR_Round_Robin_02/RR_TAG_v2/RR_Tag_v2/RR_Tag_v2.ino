#include <PortentaUWBShield.h>
#include <PurePursuit.h>
#include <math.h>


/**
 * 
 */

// Anchor Locations in Centimeters (x,y) z=0 
const float Anchor1_x=0;// cm 
const float Anchor1_y=0; // cm 
const float Anchor2_x=0; // cm 
const float Anchor2_y=467; // cm 
const float Anchor3_x=506; // cm 
const float Anchor3_y=0; // cm 
// Initialize Distance Variables
int dist_1 = 0;
int dist_2 = 0;
int dist_3 = 0; 
// Anchor received state variables
bool anchor1_received = false;
bool anchor2_received = false;
bool anchor3_received = false;

//previous position state variable 
float currentX_global = 0.0f;// cm 
float currentY_global = 0.0f;// cm 
bool prev_valid = false;

// Heading State Variable 
float Azimuth = 0.0f; // rad

// constants 
static constexpr int HALF_CIRCULAR_BUFFER_SIZE = 5;
static constexpr int CIRCULAR_BUFFER_SIZE = HALF_CIRCULAR_BUFFER_SIZE*2;
const float MinMovement = 5.0f; // cm 
const float minMovement_sq=MinMovement*MinMovement; // minimum movement squared
float x_circular_buffer[CIRCULAR_BUFFER_SIZE];
float y_circular_buffer[CIRCULAR_BUFFER_SIZE];
int head_index = 0;
int tail_index =  CIRCULAR_BUFFER_SIZE - 1;
bool inRangingHandler = false;

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
  return; // wait for all data
}else 
{
// reseting bools
  anchor1_received = false;
  anchor2_received = false;
  anchor3_received = false;

// ------------ Trilateration Calculation ------------ //
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

// ------------ End Trilateration Calculation ------------ //

  // Serial.print("Position: ");
  // Serial.print(x);
  // Serial.print(", ");
  // Serial.println(y);


// populate circular buffer for n valid data points 
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
head_index++; // head_index + 1 
if (head_index == CIRCULAR_BUFFER_SIZE) { // when index = to buffer size reset to zero
  head_index = 0;
}
tail_index++;
if (tail_index == CIRCULAR_BUFFER_SIZE) {
  tail_index = 0;
} 
// Set head_index to current postion
x_circular_buffer[head_index] = x; 
y_circular_buffer[head_index] = y;
// ------------ End Circular Buffer Advance ------------ //

float currentX=0.0f;
float currentY=0.0f;
float prevX=0.0f;
float prevY=0.0f;



  // currentX = x;
  // currentY = y;
  // // Assign prevX to the tail of buffer 
  // prevX = x_circular_buffer[tail_index];
  // prevY = y_circular_buffer[tail_index];


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
 // weighted average
     currentX += weights[i]*x_circular_buffer[index];
     currentY += weights[i]*y_circular_buffer[index];

    index--;
  }

  index = tail_index;
for (int i = 0; i < HALF_CIRCULAR_BUFFER_SIZE; i++) {
  if (index >= CIRCULAR_BUFFER_SIZE) {
    index = index - CIRCULAR_BUFFER_SIZE;
  }
   // weighted average
    prevX += weights[i]*x_circular_buffer[index];
    prevY += weights[i]*y_circular_buffer[index];
  index++;
}
currentX_global = currentX;
currentY_global = currentY;
  Serial.print("( ");
  Serial.println(currentX);
  Serial.print(" , ");
  Serial.println(currentY);
  Serial.print(") ");
// ------------ End Weighted Average ------------ //

// ------------ Heading Calculation ------------ //

// Calculate the differnce in position from previous point
    float dx = currentX - prevX;
    float dy = currentY - prevY;
// Compare the dx^2+dy^2 to the distance to flag invalid headings
    float dist_sq = dx*dx + dy*dy;
    if (dist_sq >= minMovement_sq) { // heading is valid calcualte azimuth
      Azimuth = atan2f(dy, dx);
     }else{
  Serial.print("Azimuth invalid  ");
  inRangingHandler = false;
  return;
       }
   
//convert to degrees 
Azimuth=Azimuth*(180.0/PI); 
  // print the heading
  Serial.print("Heading: ");
  Serial.println(Azimuth);
  
    
// ------------ End Heading Calculation ------------ //

}
}

  inRangingHandler = false; // reset to 0 
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

}

void loop() {

  #if defined(ARDUINO_PORTENTA_C33)
  /* Only the Portenta C33 has an RGB LED. */
  digitalWrite(LEDR, !digitalRead(LEDR));
#endif

  delay(1000);
}