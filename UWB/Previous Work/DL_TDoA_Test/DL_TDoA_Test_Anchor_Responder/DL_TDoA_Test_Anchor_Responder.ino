/**
 * DL-TDoA Test Anchor Responder
 * 
 * This script configures an Arduino Portenta C33 with UWB Shield as a DL-TDoA Anchor Responder.
 * This is a simplified test setup with 2 anchors.
 * 
 * Configuration:
 * - Anchor Position: (1.8, 0) meters
 * - MAC Address: 0xFF, 0xFF
 * - Session ID: 0x87654321 (must match initiator)
 * - Channel: 9
 */

#include <PortentaUWBShield.h>
#include "uwbapps/UWBDltdoaResponder.hpp"
#include "uwbapps/UWBAnchorCoordinates.hpp"
#include "uwbapps/UWBMacAddressList.hpp"
#include "uwbapps/UWBActiveRounds.hpp"

// Anchor configuration
const uint32_t SESSION_ID = 0x87654321;
const uint8_t ANCHOR_MAC[] = {0xFF, 0xFF};

// Anchor position in centimeters (2D: x, y, z=0)
// Position: (1.8, 0) meters = (180, 0) centimeters
const int32_t ANCHOR_X = 180;    // X position in centimeters (1.8 meters)
const int32_t ANCHOR_Y = 0;      // Y position in centimeters
const int32_t ANCHOR_Z = 0;      // Z position in centimeters (set to 0 for 2D)

// Initiator anchor MAC address
const uint8_t INITIATOR_MAC[] = {0xEE, 0xEE};

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) {
    delay(10);
  }

#if defined(ARDUINO_PORTENTA_C33)
  pinMode(LEDG, OUTPUT);
  digitalWrite(LEDG, LOW);
#endif

  Serial.println("========================================");
  Serial.println("DL-TDoA Test Anchor Responder");
  Serial.println("========================================");
  Serial.print("Anchor Position: (");
  Serial.print(ANCHOR_X / 100.0, 2);  // Convert cm to meters for display
  Serial.print(", ");
  Serial.print(ANCHOR_Y / 100.0, 2);
  Serial.print(", ");
  Serial.print(ANCHOR_Z / 100.0, 2);
  Serial.println(") meters");
  Serial.print("MAC Address: 0x");
  Serial.print(ANCHOR_MAC[0], HEX);
  Serial.print(ANCHOR_MAC[1], HEX);
  Serial.println();
  Serial.print("Session ID: 0x");
  Serial.println(SESSION_ID, HEX);
  Serial.println("========================================\n");

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

  // Set anchor coordinates
  UWBAnchorCoordinates coords;
  coords.setCoordinatesAvailable(true);
  coords.setCoordinateSystem(false);  // false = relative coordinates
  coords.setRelativeCoordinates(ANCHOR_X, ANCHOR_Y, ANCHOR_Z);

  // Configure active ranging rounds
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

  // Wait for initiator to start ranging session
  // In DL-TDoA, responder must wait for initiator to start first
  Serial.println("Waiting for initiator to start ranging session...");
  delay(8000);  // Increased delay to give initiator time to fully start

  // Retry mechanism: keep trying to start until initiator is ready
  int maxRetries = 15;
  int retryCount = 0;
  // Reuse status variable from init() above

  Serial.println("Attempting to start ranging...");
  do {
    if (retryCount > 0) {
      Serial.print("Retry attempt ");
      Serial.print(retryCount);
      Serial.print(" of ");
      Serial.println(maxRetries);
      delay(3000);  // Wait between retries
    }
    
    Serial.print("Calling responder.start()... ");
    status = responder.start();
    Serial.print("Result: ");
    Serial.println((int)status);
    
    retryCount++;
    
    if (status != uwb::Status::SUCCESS && retryCount < maxRetries) {
      Serial.print("Failed to start (status: ");
      Serial.print((int)status);
      Serial.print(" = ");
      // Status 2 typically means INVALID_PARAM or NOT_READY
      if ((int)status == 2) {
        Serial.print("INVALID_PARAM/NOT_READY");
      }
      Serial.println("), will retry...");
    }
  } while (status != uwb::Status::SUCCESS && retryCount < maxRetries);

  if (status != uwb::Status::SUCCESS) {
    Serial.print("Failed to start ranging after ");
    Serial.print(maxRetries);
    Serial.print(" attempts with status: ");
    Serial.println((int)status);
    Serial.println("Status 2 typically indicates:");
    Serial.println("  - INVALID_PARAM: Configuration issue");
    Serial.println("  - NOT_READY: Initiator not ready");
    Serial.println("  - Check that initiator has started ranging");
    Serial.println("  - Verify MAC addresses and session ID match");
    while (1) delay(1000);
  }
  
  Serial.println("Ranging started - Anchor Responder is active");
  Serial.println("Responding to initiator anchor...\n");
}

void loop() {
#if defined(ARDUINO_PORTENTA_C33)
  digitalWrite(LEDG, !digitalRead(LEDG));
#endif
  delay(1000);
}

