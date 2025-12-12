/**
 * DL-TDoA Anchor Initiator
 * 
 * This script configures an Arduino Portenta with UWB Shield as a DL-TDoA Anchor Initiator.
 * The anchor has a known position and participates in ranging sessions with other anchors.
 * 
 * Configuration:
 * - Anchor Position: (0, 0) meters (stored as 0, 0 in centimeters, can be modified)
 * - MAC Address: 0xAA, 0xAA
 * - Session ID: 0x12345678
 * - Channel: 9
 */
#include <ArduinoBLE.h>
#include <PortentaUWBShield.h>
#include <uwbapps/UWBDltdoaInitiator.hpp>
#include <uwbapps/UWBAnchorCoordinates.hpp>
#include <uwbapps/UWBAppParamsList.hpp>
#include <uwbapps/UWBAppParamList.hpp>
#include "uwbapps/UWBMacAddressList.hpp"
#include <uwbapps/UWBActiveRounds.hpp>
#include "hal/UWB_types.hpp"

// Anchor configuration
const uint32_t SESSION_ID = 0x11223344;
const uint8_t ANCHOR_MAC[] = {0xAA, 0xAA};

// Session info notification handler
void sessionInfoHandler(uwb::SessionInfo &sessionInfo) {
  Serial.print("Session Info - Handle: 0x");
  Serial.print(sessionInfo.sessionHandle, HEX);
  Serial.print(", State: ");
  Serial.print(sessionInfo.state);
  Serial.print(", Reason: ");
  Serial.println(sessionInfo.reason_code);
}

void rangingHandler(UWBRangingData &rangingData) {
  Serial.println("Ranging data received");
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) {
    delay(10);
  }

#if defined(ARDUINO_PORTENTA_C33)
  pinMode(LEDR, OUTPUT);
  digitalWrite(LEDR, LOW);
#endif

  UWB.registerRangingCallback(rangingHandler);
  UWB.registerSessionInfoCallback(sessionInfoHandler);

  // Destination anchors (responders) MAC addresses
  uint8_t RESPONDER1_MAC[] = {0xBB, 0xBB};
  uint8_t RESPONDER2_MAC[] = {0xCC, 0xCC};
  uint8_t ROUND_INDEX = 1;

  // Anchor position in centimeters (2D: x, y, z=0)
  int ANCHOR_X = 0;
  int ANCHOR_Y = 0;
  int ANCHOR_Z = 0;

  Serial.println("DL-TDoA Anchor Initiator Starting...");
  Serial.print("Anchor Position: (");
  Serial.print(ANCHOR_X / 100.0, 2);  // Convert cm to meters for display
  Serial.print(", ");
  Serial.print(ANCHOR_Y / 100.0, 2);
  Serial.print(", ");
  Serial.print(ANCHOR_Z / 100.0, 2);
  Serial.println(") meters");

  Serial.println("UWB stack ready");

  // Create source MAC address
  UWBMacAddress srcAddr(UWBMacAddress::Size::SHORT, ANCHOR_MAC);

  // Create destination MAC addresses list
  UWBMacAddressList dstAddrs(UWBMacAddress::Size::SHORT);
  UWBMacAddress responder1Addr(UWBMacAddress::Size::SHORT, RESPONDER1_MAC);
  UWBMacAddress responder2Addr(UWBMacAddress::Size::SHORT, RESPONDER2_MAC);
  dstAddrs.add(responder1Addr);
  dstAddrs.add(responder2Addr);

  // Set anchor coordinates (using relative coordinates for 2D positioning)
  UWBAnchorCoordinates coords;
  coords.setCoordinatesAvailable(true);
  coords.setCoordinateSystem(false);  // false = relative coordinates (not WGS84)
  coords.setRelativeCoordinates(ANCHOR_X, ANCHOR_Y, ANCHOR_Z);

  // Configure active ranging rounds
  UWBActiveRounds rounds(1);

  // Create DL-TDoA Initiator session
  UWBDltdoaInitiator initiator(0x11223344, srcAddr, coords, dstAddrs, rounds);

  // Initialize UWB stack
  UWB.begin();
  Serial.println("UWB stack initialized");

  // Wait for UWB stack to be ready
  while (UWB.state() != 0) {
    delay(10);
  }

  UWBSessionManager.addSession(initiator);
  initiator.start();
  Serial.println("Session initialized in constructor");

  // Verify session handle is valid
  if (initiator.sessionID() == 0) {
    Serial.println("ERROR: Invalid session handle - init may have failed");
    Serial.println("Halting execution");
    while (1) {
      delay(1000);
    }
  }
  Serial.print("Session handle: 0x");
  Serial.println(initiator.sessionID(), HEX);

  // Add small delay to allow session to fully stabilize
  delay(100);

  Serial.println("Ranging started - Anchor Initiator is active");
  Serial.println("Listening for tags and ranging with other anchors...");
}

void loop() {
#if defined(ARDUINO_PORTENTA_C33)
  digitalWrite(LEDR, !digitalRead(LEDR));
#endif
  delay(10);
}

