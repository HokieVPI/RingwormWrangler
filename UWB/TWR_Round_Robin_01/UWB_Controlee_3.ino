#include <PortentaUWBShield.h>


/**
 * this demo shows how to setup the Arduino Stella tag as simple 
 * Two-Way Ranging Initiator/Controller
 * It expects a counterpart setup as a Responder/Controlee
 * 
 */


// handler for ranging notifications
void rangingHandler(UWBRangingData &rangingData) {
  Serial.print("GOT RANGING DATA - Type: "  );
  Serial.println(rangingData.measureType());
  if(rangingData.measureType()==(uint8_t)uwb::MeasurementType::TWO_WAY)
  {
    
    RangingMeasures twr=rangingData.twoWayRangingMeasure();
    for(int j=0;j<rangingData.available();j++)
    {

      if(twr[j].status==0 && twr[j].distance!=0xFFFF)
      {
        Serial.print("Distance: ");
        Serial.println(twr[j].distance);  
      }
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

  //define the source (this device) and destination MAC addresses, using 2-bytes MACs
  uint8_t devAddr[]={0x22,0x22};
  uint8_t destination[]={0x11,0x11};
  UWBMacAddress srcAddr(UWBMacAddress::Size::SHORT,devAddr);
  UWBMacAddress dstAddr(UWBMacAddress::Size::SHORT,destination);
  
  // register the ranging notification handler before starting
  UWB.registerRangingCallback(rangingHandler);
  
  UWB.begin(); //start the UWB stack, use Serial for the log output
  Serial.println("Starting UWB ...");
  
  //wait until the stack is initialised
  while(UWB.state()!=0)
    delay(10);

  Serial.println("Starting session ...");
  //setup a session with ID 0x11223344
  UWBRangingControlee myControlee(0x11223344, srcAddr, dstAddr);
  
  //add the session to the session manager, in case you want to manage multiple connections
  UWBSessionManager.addSession(myControlee);

  //prepare the session applying the default parameters
  myControlee.init();
  
  //start the session
  myControlee.start();

}

void loop() {
#if defined(ARDUINO_PORTENTA_C33)
  /* Only the Portenta C33 has an RGB LED. */
  digitalWrite(LEDR, !digitalRead(LEDR));
#endif
  delay(1000);
}
