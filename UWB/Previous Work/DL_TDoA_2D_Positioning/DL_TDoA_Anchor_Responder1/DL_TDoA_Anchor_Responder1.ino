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

 #include <ArduinoBLE.h>
 #include <PortentaUWBShield.h>
 #include <uwbapps/UWBDltdoaResponder.hpp>
 #include <uwbapps/UWBAnchorCoordinates.hpp>
 #include <uwbapps/UWBAppParamsList.hpp>
 #include <uwbapps/UWBAppParamList.hpp>
 #include "uwbapps/UWBMacAddressList.hpp"
 #include <uwbapps/UWBActiveRounds.hpp>
 #include "hal/UWB_types.hpp"

// Anchor configuration
const uint32_t SESSION_ID = 0x11223344;
const uint8_t ANCHOR_MAC[] = {0xBB, 0xBB};



// Initiator anchor MAC address
const uint8_t INITIATOR_MAC[] = {0xAA, 0xAA};

// Ranging round index
const uint8_t ROUND_INDEX = 1;

// Session info notification handler
void sessionInfoHandler(uwb::SessionInfo &sessionInfo) {
  Serial.print("Session Info - Handle: 0x");
  Serial.print(sessionInfo.sessionHandle, HEX);
  Serial.print(", State: ");
  Serial.print(sessionInfo.state);
  Serial.print(", Reason: ");
  Serial.println(sessionInfo.reason_code);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) {
    delay(10);
  }

#if defined(ARDUINO_PORTENTA_C33)
  pinMode(LEDG, OUTPUT);
  digitalWrite(LEDG, LOW);
#endif
  UWB.registerSessionInfoCallback(sessionInfoHandler);

  // Anchor position in centimeters (2D: x, y, z=0)
  int ANCHOR_X = 180;
  int ANCHOR_Y = 0;
  int ANCHOR_Z = 0;
  Serial.println("DL-TDoA Anchor Responder 1 Starting...");
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

  // Create destination MAC addresses list (initiator)
  UWBMacAddressList dstAddrs(UWBMacAddress::Size::SHORT);
  UWBMacAddress initiatorAddr(UWBMacAddress::Size::SHORT, INITIATOR_MAC);
  dstAddrs.add(initiatorAddr);

  // Set anchor coordinates
  UWBAnchorCoordinates coords;
  coords.setCoordinatesAvailable(true);
  coords.setCoordinateSystem(false);  // false = relative coordinates
  coords.setRelativeCoordinates(ANCHOR_X, ANCHOR_Y, ANCHOR_Z);

  // Configure active ranging rounds
  UWBActiveRounds rounds(1);

  // Create DL-TDoA Responder session
  UWBDltdoaResponder responder(SESSION_ID, srcAddr, coords, dstAddrs, rounds);

  // Initialize UWB stack
  UWB.begin();
  Serial.println("UWB stack initialized");

  // Wait for UWB stack to be ready
  while (UWB.state() != 0) {
    delay(10);
  }

  UWBSessionManager.addSession(responder);
  responder.start();
  Serial.println("Session initialized in constructor");

  // Verify session handle is valid
  if (responder.sessionID() == 0) {
    Serial.println("ERROR: Invalid session handle - init may have failed");
    Serial.println("Halting execution");
    while (1) {
      delay(1000);
    }
  }
  Serial.print("Session handle: 0x");
  Serial.println(responder.sessionID(), HEX);

  // Add small delay to allow session to fully stabilize
  delay(100);

  Serial.println("Ranging started - Anchor Responder 1 is active");
}

void loop() {
#if defined(ARDUINO_PORTENTA_C33)
  digitalWrite(LEDG, !digitalRead(LEDG));
#endif
  delay(1000);
}

