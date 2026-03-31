#include <PortentaUWBShield.h>

void rangingHandler(UWBRangingData &rangingData) {
  Serial.print("GOT RANGING DATA - Type: ");
  Serial.println(rangingData.measureType());
  if (rangingData.measureType() == (uint8_t)uwb::MeasurementType::TWO_WAY) {
    RangingMeasures twr = rangingData.twoWayRangingMeasure();
    for (int j = 0; j < rangingData.available(); j++) {
      if (twr[j].status == 0 && twr[j].distance != 0xFFFF) {
        Serial.print("Distance: ");
        Serial.println(twr[j].distance);
      }
    }
  }
}

void setup() {
  Serial.begin(115200);

#if defined(ARDUINO_PORTENTA_C33)
  pinMode(LEDR, OUTPUT);
  digitalWrite(LEDR, LOW);
#endif

  // ← only these two lines differ from your other anchors
  uint8_t devAddr[]    = {0x55, 0x55};  // anchor 4's unique address
  uint8_t destination[] = {0x11, 0x11}; // always points back to the tag

  UWBMacAddress srcAddr(UWBMacAddress::Size::SHORT, devAddr);
  UWBMacAddress dstAddr(UWBMacAddress::Size::SHORT, destination);

  UWB.registerRangingCallback(rangingHandler);

  UWB.begin();
  Serial.println("Starting UWB ...");

  while (UWB.state() != 0)
    delay(10);

  Serial.println("Starting session ...");

  UWBRangingControlee myControlee(0x11223344, srcAddr, dstAddr);
  myControlee.rangingParams.multiNodeMode(uwb::MultiNodeMode::MULTICAST);
  UWBSessionManager.addSession(myControlee);
  myControlee.init();
  myControlee.start();
}

void loop() {
#if defined(ARDUINO_PORTENTA_C33)
  digitalWrite(LEDR, !digitalRead(LEDR));
#endif
  delay(1000);
}