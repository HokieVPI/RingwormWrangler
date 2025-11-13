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

#include <PortentaUWBShield.h>
#include "uwbapps/UWBDltdoaInitiator.hpp"
#include "uwbapps/UWBAnchorCoordinates.hpp"
#include "uwbapps/UWBMacAddressList.hpp"
#include "uwbapps/UWBActiveRounds.hpp"

// Anchor configuration
const uint32_t SESSION_ID = 0x12345678;
const uint8_t ANCHOR_MAC[] = {0xAA, 0xAA};

// Anchor position in centimeters (2D: x, y, z=0)
// Note: The coordinate system uses integers, so we use centimeters
// For meters, multiply by 100 (e.g., 2.2 meters = 220 centimeters)
// Modify these values to match your anchor's physical position
const int32_t ANCHOR_X = 0;    // X position in centimeters (can be negative)
const int32_t ANCHOR_Y = 0;    // Y position in centimeters (can be negative)
const int32_t ANCHOR_Z = 0;    // Z position in centimeters (set to 0 for 2D)

// Destination anchors (responders) MAC addresses
const uint8_t RESPONDER1_MAC[] = {0xBB, 0xBB};  // Anchor 2
const uint8_t RESPONDER2_MAC[] = {0xCC, 0xCC};  // Anchor 3

// Ranging round index (typically 0 for the first round)
const uint8_t ROUND_INDEX = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) {
    delay(10);
  }

#if defined(ARDUINO_PORTENTA_C33)
  pinMode(LEDR, OUTPUT);
  digitalWrite(LEDR, LOW);
#endif

  Serial.println("DL-TDoA Anchor Initiator Starting...");
  Serial.print("Anchor Position: (");
  Serial.print(ANCHOR_X / 100.0, 2);  // Convert cm to meters for display
  Serial.print(", ");
  Serial.print(ANCHOR_Y / 100.0, 2);
  Serial.print(", ");
  Serial.print(ANCHOR_Z / 100.0, 2);
  Serial.println(") meters");

  // Initialize UWB stack
  UWB.begin(Serial, uwb::LogLevel::UWB_INFO_LEVEL);
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
  // Note: The phActiveRoundsConfig_t type is not exposed in the library API
  // The UWBActiveRounds class handles this internally
  // For now, create an empty rounds object - the DL-TDoA classes may handle configuration internally
  UWBActiveRounds rounds(1);
  
  // TODO: If the library requires active rounds configuration, you may need to:
  // 1. Check if phActiveRoundsConfig_t is defined in the HAL implementation
  // 2. Use a different API if available
  // 3. Contact the library maintainers for the correct way to configure DL-TDoA rounds

  // Create DL-TDoA Initiator session
  UWBDltdoaInitiator initiator(SESSION_ID, srcAddr, coords, dstAddrs, rounds);
  
  // Add session to session manager
  UWBSessionManager.addSession(initiator);
  
  // Initialize the session
  uwb::Status status = initiator.init();
  if (status != uwb::Status::SUCCESS) {
    Serial.print("Session initialization failed with status: ");
    Serial.println((int)status);
    while (1) delay(1000);
  }
  Serial.println("Session initialized successfully");

  // Start ranging
  status = initiator.start();
  if (status != uwb::Status::SUCCESS) {
    Serial.print("Failed to start ranging with status: ");
    Serial.println((int)status);
    while (1) delay(1000);
  }
  Serial.println("Ranging started - Anchor Initiator is active");
  Serial.println("Listening for tags and ranging with other anchors...");
}

void loop() {
#if defined(ARDUINO_PORTENTA_C33)
  digitalWrite(LEDR, !digitalRead(LEDR));
#endif
  delay(1000);
}

