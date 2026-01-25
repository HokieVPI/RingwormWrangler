#include <PortentaUWBShield.h>
#include <math.h>

/**
 * AI_test_RR - Stella Controller/Tag
 *
 * Hardware:
 * - Arduino Stella: controller (initiator) + tag
 * - 3x Portenta C33 + UWB Shield: anchors (controlees/responders)
 *
 * Output (ONLY):
 *   x,y
 *
 * Notes:
 * - Uses one-to-many ranging (time scheduled DS-TWR) and computes (x,y) from 3 ranges.
 * - Assumes `twr[j].distance` is in millimeters -> meters via 0.001f.
 */

// ---------- Fixed configuration ----------
static constexpr uint32_t SESSION_ID = 0x11223344;

// 2-byte SHORT MAC addresses (duplicated bytes per your request)
static constexpr uint8_t TAG_MAC[2] = {0x11, 0x11};
static constexpr uint8_t A1_MAC[2]  = {0x22, 0x22};
static constexpr uint8_t A2_MAC[2]  = {0x33, 0x33};
static constexpr uint8_t A3_MAC[2]  = {0x44, 0x44};

// Anchor coordinates (meters)
static constexpr float A1_X = 0.0f, A1_Y = 0.0f;
static constexpr float A2_X = 4.0f, A2_Y = 0.0f;
static constexpr float A3_X = 2.0f, A3_Y = 2.0f;

// Distance scaling (adjust if your library reports centimeters instead of millimeters)
static constexpr float DISTANCE_TO_METERS = 0.001f; // mm -> m

// Freshness window: only compute (x,y) if all 3 distances updated recently
static constexpr uint32_t MAX_AGE_MS = 500;

// ---------- State ----------
static volatile float r1_m = -1.0f;
static volatile float r2_m = -1.0f;
static volatile float r3_m = -1.0f;
static volatile uint32_t r1_ms = 0;
static volatile uint32_t r2_ms = 0;
static volatile uint32_t r3_ms = 0;

static inline bool isMacMatchShort(const uint8_t peer_addr[8], const uint8_t mac2[2]) {
  return (peer_addr[0] == mac2[0]) && (peer_addr[1] == mac2[1]);
}

static inline void updateRange(const uint8_t peer_addr[8], uint16_t dist_raw) {
  const float d_m = (float)dist_raw * DISTANCE_TO_METERS;
  const uint32_t now = millis();

  if (isMacMatchShort(peer_addr, A1_MAC)) {
    r1_m = d_m;
    r1_ms = now;
  } else if (isMacMatchShort(peer_addr, A2_MAC)) {
    r2_m = d_m;
    r2_ms = now;
  } else if (isMacMatchShort(peer_addr, A3_MAC)) {
    r3_m = d_m;
    r3_ms = now;
  }
}

static inline bool rangesFresh(float& o_r1, float& o_r2, float& o_r3) {
  const uint32_t now = millis();
  const float _r1 = r1_m, _r2 = r2_m, _r3 = r3_m;
  const uint32_t _r1ms = r1_ms, _r2ms = r2_ms, _r3ms = r3_ms;

  if (_r1 <= 0.0f || _r2 <= 0.0f || _r3 <= 0.0f) return false;
  if ((now - _r1ms) > MAX_AGE_MS) return false;
  if ((now - _r2ms) > MAX_AGE_MS) return false;
  if ((now - _r3ms) > MAX_AGE_MS) return false;

  o_r1 = _r1; o_r2 = _r2; o_r3 = _r3;
  return true;
}

// Trilateration for anchors at (0,0), (4,0), (2,2)
static inline bool trilaterate_3anchors(float r1, float r2, float r3, float& x, float& y) {
  // x = (r1^2 - r2^2 + 16) / 8  (since 4^2 = 16)
  const float r1_2 = r1 * r1;
  const float r2_2 = r2 * r2;
  const float r3_2 = r3 * r3;

  x = (r1_2 - r2_2 + (A2_X * A2_X)) / (2.0f * A2_X); // (.. + 16) / 8

  // From (A3 - A1): x + y = (8 - r3^2 + r1^2) / 4  (since A3=(2,2) => constant 8)
  const float xy = ((A3_X * A3_X + A3_Y * A3_Y) - r3_2 + r1_2) / (2.0f * A3_X); // (8 - r3^2 + r1^2) / 4
  y = xy - x;

  // Basic sanity: finite numbers
  if (!isfinite(x) || !isfinite(y)) return false;
  return true;
}

// handler for ranging notifications
void rangingHandler(UWBRangingData& rangingData) {
  if (rangingData.measureType() != (uint8_t)uwb::MeasurementType::TWO_WAY) return;

  RangingMeasures twr = rangingData.twoWayRangingMeasure();
  for (int j = 0; j < rangingData.available(); j++) {
    if (twr[j].status == 0 && twr[j].distance != 0xFFFF && twr[j].distance != 0) {
      updateRange(twr[j].peer_addr, twr[j].distance);
    }
  }

  float r1, r2, r3;
  if (!rangesFresh(r1, r2, r3)) return;

  float x, y;
  if (!trilaterate_3anchors(r1, r2, r3, x, y)) return;

  // ONLY output: CSV x,y
  Serial.print(x, 3);
  Serial.print(',');
  Serial.println(y, 3);
}

void setup() {
  Serial.begin(115200);

  // Register callback before starting UWB stack
  UWB.registerRangingCallback(rangingHandler);

  // Silence UWB internal logging so Serial output stays ONLY x,y
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
  // No periodic prints here (Serial output is only produced on valid ranging updates)
  delay(10);
}

