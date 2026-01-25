#include <PortentaUWBShield.h>

/**
 * AI_test_RR - Anchor 2 (Portenta C33 + UWB Shield)
 *
 * Output:
 * - None (the Stella prints x,y).
 */

static constexpr uint32_t SESSION_ID = 0x11223344;

// 2-byte SHORT MAC addresses (duplicated bytes)
static constexpr uint8_t ANCHOR_MAC[2] = {0x33, 0x33}; // Anchor 2
static constexpr uint8_t TAG_MAC[2]    = {0x11, 0x11}; // Stella tag/controller

void setup() {
  Serial.begin(115200);

  UWB.begin(Serial, uwb::LogLevel::UWB_SILENT_LEVEL);
  while (UWB.state() != 0) delay(10);

#if defined(ARDUINO_PORTENTA_C33)
  pinMode(LEDR, OUTPUT);
  digitalWrite(LEDR, LOW);
#endif

  UWBMacAddress srcAddr(UWBMacAddress::Size::SHORT, ANCHOR_MAC);
  UWBMacAddress dstAddr(UWBMacAddress::Size::SHORT, TAG_MAC);

  UWBRangingControlee myControlee(SESSION_ID, srcAddr, dstAddr);
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

