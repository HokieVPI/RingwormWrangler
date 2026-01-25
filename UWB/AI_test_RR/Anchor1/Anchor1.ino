#include <PortentaUWBShield.h>

/**
 * AI_test_RR - Anchor 1 (Portenta C33 + UWB Shield)
 *
 * Stationary anchor/controlee responder for one-to-many ranging with a Stella controller/tag.
 *
 * Output:
 * - None (keeps Serial quiet; the Stella prints x,y).
 */

static constexpr uint32_t SESSION_ID = 0x11223344;

// 2-byte SHORT MAC addresses (duplicated bytes)
static constexpr uint8_t ANCHOR_MAC[2] = {0x22, 0x22}; // Anchor 1
static constexpr uint8_t TAG_MAC[2]    = {0x11, 0x11}; // Stella tag/controller

void setup() {
  Serial.begin(115200);

  // Silence UWB internal logging to keep Serial quiet
  UWB.begin(Serial, uwb::LogLevel::UWB_SILENT_LEVEL);
  while (UWB.state() != 0) delay(10);

#if defined(ARDUINO_PORTENTA_C33)
  pinMode(LEDR, OUTPUT);
  digitalWrite(LEDR, LOW);
#endif

  UWBMacAddress srcAddr(UWBMacAddress::Size::SHORT, ANCHOR_MAC);
  UWBMacAddress dstAddr(UWBMacAddress::Size::SHORT, TAG_MAC);

  UWBRangingControlee myControlee(SESSION_ID, srcAddr, dstAddr);

  // Controller uses one-to-many; ensure controlee matches multicast multi-node mode
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

