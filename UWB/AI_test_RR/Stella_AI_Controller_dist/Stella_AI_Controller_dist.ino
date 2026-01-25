#include <PortentaUWBShield.h>

/**
 * Stella_AI_Controller_dist
 *
 * Stella = controller/initiator + tag.
 * 3x Portenta anchors = controlees/responders.
 *
 * Output (ONLY, fixed 10 Hz):
 *   dA1,dA2,dA3
 *
 * Where A1/A2/A3 correspond to MACs:
 *  - A1: 0x22 0x22
 *  - A2: 0x33 0x33
 *  - A3: 0x44 0x44
 *
 * Notes:
 * - Assumes `twr[j].distance` is millimeters; converted to meters via 0.001.
 */

// ---------- Fixed configuration ----------
static constexpr uint32_t SESSION_ID = 0x11223344;

static constexpr uint8_t TAG_MAC[2] = {0x11, 0x11};
static constexpr uint8_t A1_MAC[2]  = {0x22, 0x22};
static constexpr uint8_t A2_MAC[2]  = {0x33, 0x33};
static constexpr uint8_t A3_MAC[2]  = {0x44, 0x44};

static constexpr float DISTANCE_TO_METERS = 0.001f; // mm -> m (adjust if needed)

static constexpr uint32_t PRINT_PERIOD_MS = 100; // 10 Hz

// ---------- State ----------
static volatile float d1_m = -1.0f;
static volatile float d2_m = -1.0f;
static volatile float d3_m = -1.0f;

static inline bool isMacMatchShort(const uint8_t peer_addr[8], const uint8_t mac2[2]) {
  return (peer_addr[0] == mac2[0]) && (peer_addr[1] == mac2[1]);
}

static inline void updateDistance(const uint8_t peer_addr[8], uint16_t dist_raw) {
  const float d_m = (float)dist_raw * DISTANCE_TO_METERS;

  if (isMacMatchShort(peer_addr, A1_MAC)) {
    d1_m = d_m;
  } else if (isMacMatchShort(peer_addr, A2_MAC)) {
    d2_m = d_m;
  } else if (isMacMatchShort(peer_addr, A3_MAC)) {
    d3_m = d_m;
  }
}

// handler for ranging notifications
void rangingHandler(UWBRangingData& rangingData) {
  if (rangingData.measureType() != (uint8_t)uwb::MeasurementType::TWO_WAY) return;

  RangingMeasures twr = rangingData.twoWayRangingMeasure();
  for (int j = 0; j < rangingData.available(); j++) {
    if (twr[j].status == 0 && twr[j].distance != 0xFFFF && twr[j].distance != 0) {
      updateDistance(twr[j].peer_addr, twr[j].distance);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Register callback before starting UWB stack
  UWB.registerRangingCallback(rangingHandler);

  // Silence UWB internal logging so Serial output stays ONLY distances
  UWB.begin(Serial, uwb::LogLevel::UWB_SILENT_LEVEL);
  while (UWB.state() != 0) delay(10);

  UWBMacAddress srcAddr(UWBMacAddress::Size::SHORT, TAG_MAC);

  UWBMacAddress dst1(UWBMacAddress::Size::SHORT, A1_MAC);
  UWBMacAddress dst2(UWBMacAddress::Size::SHORT, A2_MAC);
  UWBMacAddress dst3(UWBMacAddress::Size::SHORT, A3_MAC);

  UWBMacAddressList dest(UWBMacAddress::Size::SHORT);
  dest.add(dst1);
  dest.add(dst2);
  dest.add(dst3);

  UWBRangingOneToMany controller(SESSION_ID, srcAddr, dest);
  UWBSessionManager.addSession(controller);

  controller.init();
  controller.start();
}

void loop() {
  static uint32_t nextPrintMs = 0;
  const uint32_t now = millis();

  if ((int32_t)(now - nextPrintMs) >= 0) {
    nextPrintMs = now + PRINT_PERIOD_MS;

    float a1, a2, a3;
    noInterrupts();
    a1 = d1_m; a2 = d2_m; a3 = d3_m;
    interrupts();

    // ONLY output: CSV d1,d2,d3 (meters)
    Serial.print(a1, 3);
    Serial.print(',');
    Serial.print(a2, 3);
    Serial.print(',');
    Serial.println(a3, 3);
  }
}

