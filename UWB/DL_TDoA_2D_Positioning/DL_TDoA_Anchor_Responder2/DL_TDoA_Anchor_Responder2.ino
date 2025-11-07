/**
 * DL-TDoA Anchor Responder 2
 * 
 * This script configures an Arduino Portenta with UWB Shield as a DL-TDoA Anchor Responder.
 * This is the second responder anchor with a known position.
 * 
 * Configuration:
 * - Anchor Position: (2.5, 4.33) meters (can be modified - example: equilateral triangle)
 * - MAC Address: 0xCC, 0xCC
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
const uint8_t ANCHOR_MAC[] = {0xCC, 0xCC};

// Anchor position in meters (2D: x, y, z=0)
// Modify these values to match your anchor's physical position
// Example: For an equilateral triangle with 5m sides:
// Anchor 1: (0, 0)
// Anchor 2: (5, 0)
// Anchor 3: (2.5, 4.33) - height of equilateral triangle = side * sqrt(3)/2
const int32_t ANCHOR_X = 1.16;    // X position in M
const int32_t ANCHOR_Y = 2.00;    // Y position in M 
const int32_t ANCHOR_Z = 0;      // Z position in M (set to 0 for 2D)

// Note: The coordinate system uses integers, so we use centimeters
// For meters, multiply by 100

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
  pinMode(LEDB, OUTPUT);
  digitalWrite(LEDB, LOW);
#endif

  Serial.println("DL-TDoA Anchor Responder 2 Starting...");
  Serial.print("Anchor Position: (");
  Serial.print(ANCHOR_X );
  Serial.print(", ");
  Serial.print(ANCHOR_Y);
  Serial.print(", ");
  Serial.print(ANCHOR_Z);
  Serial.println(") meters");

  // Initialize UWB stack
  UWB.begin(Serial, uwb::LogLevel::UWB_INFO_LEVEL);
  Serial.println("UWB stack initialized");

  // Wait for UWB stack to be ready
  while (UWB.state() != 0) {
    delay(10);
  }
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
  // Note: The phActiveRoundsConfig_t type is not exposed in the library API
  // The UWBActiveRounds class handles this internally
  UWBActiveRounds rounds(1);

  // Create DL-TDoA Responder session
  UWBDltdoaResponder responder(SESSION_ID, srcAddr, coords, dstAddrs, rounds);
  
  // Add session to session manager
  UWBSessionManager.addSession(responder);
  
  // Initialize the session
  uwb::Status status = responder.init();
  if (status != uwb::Status::SUCCESS) {
    Serial.print("Session initialization failed with status: ");
    Serial.println((int)status);
    while (1) delay(1000);
  }
  Serial.println("Session initialized successfully");

  // Start ranging
  status = responder.start();
  if (status != uwb::Status::SUCCESS) {
    Serial.print("Failed to start ranging with status: ");
    Serial.println((int)status);
    while (1) delay(1000);
  }
  Serial.println("Ranging started - Anchor Responder 2 is active");
}

void loop() {
#if defined(ARDUINO_PORTENTA_C33)
  digitalWrite(LEDB, !digitalRead(LEDB));
#endif
  delay(1000);
}

