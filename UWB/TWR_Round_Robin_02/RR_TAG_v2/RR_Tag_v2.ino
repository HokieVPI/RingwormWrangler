#include <PortentaUWBShield.h>


/**
 * This Script claculates the position of an Arduino Protenta Tag based 
 * on the distances to three anchors. Using multicast to range with 3+ anchors
 * UWB Ranging Controller (one-to-many)*
 * It expects multiple counterparts setup as Responders/Controlees*
 */

// Anchor Locations in Centimeters (x,y) z=0 
const float Anchor1_x=0;
const float Anchor1_y=0;
const float Anchor2_x=1000; 
const float Anchor2_y=0; 
const float Anchor3_x=1000; 
const float Anchor3_y=1000; 

bool anchor1_received = false;
bool anchor2_received = false;
bool anchor3_received = false;


// handler for ranging notifications
void rangingHandler(UWBRangingData &rangingData) {

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

  if (fabs(det) < 1e-6) {
    Serial.println("Error: Anchors are collinear, cannot calculate position");
    return;
  }
  float x = (C*E - F*B) / det;
  float y = (A*F - C*D) / det;
  Serial.print("Position: ");
  Serial.print(x);
  Serial.print(", ");
  Serial.println(y);
}
}
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