/**
 * DL-TDoA Anchor Responder 1
 * 
 * This script configures an Arduino Portenta with UWB Shield as a DL-TDoA Anchor Responder.
 * This is the first responder anchor with a known position.
 * 
 * Configuration:
 * - Anchor Position: (5, 0) meters (can be modified)
 * - MAC Address: 0xBB, 0xBB
 * - Session ID: 0x12345678 (must match initiator)
 * - Channel: 9
 */

#include <PortentaUWBShield.h>
#include "uwbapps/UWBDltdoaResponder.hpp"
#include "uwbapps/UWBAnchorCoordinates.hpp"
#include "uwbapps/UWBMacAddressList.hpp"
#include "uwbapps/UWBActiveRounds.hpp"

// Anchor configuration
const uint32_t SESSION_ID = 0x12345678;
const uint8_t ANCHOR_MAC[] = {0xBB, 0xBB};

// Anchor position in centimeters (2D: x, y, z=0)
// Note: The coordinate system uses integers, so we use centimeters
// For meters, multiply by 100 (e.g., 2.2 meters = 220 centimeters)
// Modify these values to match your anchor's physical position
const int32_t ANCHOR_X = 180;    // X position in centimeters (2.2 meters)
const int32_t ANCHOR_Y = 0;      // Y position in centimeters
const int32_t ANCHOR_Z = 0;      // Z position in centimeters (set to 0 for 2D)

// Initiator anchor MAC address
const uint8_t INITIATOR_MAC[] = {0xAA, 0xAA};

// Ranging round index
const uint8_t ROUND_INDEX = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) {
    delay(10);
  }

#if defined(ARDUINO_PORTENTA_C33)
  pinMode(LEDG, OUTPUT);
  digitalWrite(LEDG, LOW);
#endif

  Serial.println("DL-TDoA Anchor Responder 1 Starting...");
  Serial.print("Anchor Position: (");
  Serial.print(ANCHOR_X / 100.0, 2);  // Convert cm to meters for display
  Serial.print(", ");
  Serial.print(ANCHOR_Y / 100.0, 2);
  Serial.print(", ");
  Serial.print(ANCHOR_Z / 100.0, 2);
  Serial.println(") meters");

  // Initialize UWB stack
  UWB.begin();
  Serial.println("UWB stack initialized");

  // Add delay to allow hardware to stabilize
  delay(500);

  // Wait for UWB stack to be ready with timeout
  unsigned long startTime = millis();
  while (UWB.state() != 0 && (millis() - startTime) < 10000) {
    delay(10);
  }
  
  if (UWB.state() != 0) {
    Serial.println("ERROR: UWB stack failed to initialize within timeout!");
    Serial.print("UWB state: ");
    Serial.println(UWB.state());
    while (1) {
      delay(1000);
    }
  }
  Serial.println("UWB stack ready");

  // Create source MAC address
  UWBMacAddress srcAddr(UWBMacAddress::Size::SHORT, ANCHOR_MAC);

  // Create destination MAC addresses list (initiator)
  UWBMacAddressList dstAddrs(UWBMacAddress::Size::SHORT);
  UWBMacAddress initiatorAddr(UWBMacAddress::Size::SHORT, INITIATOR_MAC);
  dstAddrs.add(initiatorAddr);
  // WARNING 7: MAC addresses in list may not be directly used in the constructor loop.
  // The UWB stack may use session ID and MAC addresses from ranging parameters instead.
  // If ranging fails, this may need investigation.

  // Set anchor coordinates
  UWBAnchorCoordinates coords;
  coords.setCoordinatesAvailable(true);
  coords.setCoordinateSystem(false);  // false = relative coordinates
  coords.setRelativeCoordinates(ANCHOR_X, ANCHOR_Y, ANCHOR_Z);

  // Configure active ranging rounds
  // Note: The phActiveRoundsConfig_t type is not exposed in the library API
  // The UWBActiveRounds class handles this internally
  // WARNING 4: Empty rounds may be acceptable if the UWB stack uses default round configuration.
  // If ranging fails, explicit round configuration may be needed.
  UWBActiveRounds rounds(1);

  // Create DL-TDoA Responder session
  UWBDltdoaResponder responder(SESSION_ID, srcAddr, coords, dstAddrs, rounds);
  
  // Add session to session manager (NOTE: Session manager creates a new incomplete object,
  // so we must use the original 'responder' object for all operations - see Issue 2 workaround)
  UWBSessionManager.addSession(responder);
  
  // Note: The UWBDltdoaResponder constructor already calls init() internally (line 69 in UWBDltdoaResponder.hpp),
  // so we should NOT call init() again here to avoid double initialization errors.
  Serial.println("Session initialized in constructor");

  // Start ranging (use original object, not session manager's stored object)
  uwb::Status status = responder.start();
  if (status != uwb::Status::SUCCESS) {
    Serial.print("Failed to start ranging with status: ");
    Serial.println((int)status);
    while (1) delay(1000);
  }
  Serial.println("Ranging started - Anchor Responder 1 is active");
}

void loop() {
#if defined(ARDUINO_PORTENTA_C33)
  digitalWrite(LEDG, !digitalRead(LEDG));
#endif
  delay(1000);
}

