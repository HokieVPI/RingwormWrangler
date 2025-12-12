/**
 * DL-TDoA Test Tag
 * 
 * This script configures an Arduino Portenta C33 with UWB Shield as a DL-TDoA Tag.
 * The tag listens to ranging sessions between anchors and displays measurement data.
 * 
 * This is a simplified test setup that displays measurements without full position calculation.
 * 
 * Configuration:
 * - MAC Address: 0x11, 0x11
 * - Session ID: 0x87654321 (must match anchors)
 * - Channel: 9
 */

#include <PortentaUWBShield.h>
#include "uwbapps/UWBDltdoaTag.hpp"
#include <math.h>

// Tag configuration
const uint32_t SESSION_ID = 0x87654321;
const uint8_t TAG_MAC[] = {0x11, 0x11};

// Known anchor positions (in meters) - for reference only
// These match the positions configured in the anchor scripts
struct AnchorPosition {
  float x;  // X position in meters
  float y;  // Y position in meters
  uint8_t mac[2];  // MAC address
};

// Define anchor positions (for display purposes)
const AnchorPosition ANCHORS[] = {
  {0.0, 0.0, {0xEE, 0xEE}},      // Anchor 1 (Initiator)
  {1.8, 0.0, {0xFF, 0xFF}}       // Anchor 2 (Responder)
};

const uint8_t NUM_ANCHORS = 2;

// Timestamp unit: 2^(-7) of 499.2 MHz chipping period ≈ 15.65 ps
const double TIMESTAMP_UNIT_SEC = 15.65e-12;

// Speed of light in meters per second
const float SPEED_OF_LIGHT = 299792458.0;

// Ranging round index list
uint8_t roundIndexList[] = {0};

// Ranging data handler
void rangingHandler(UWBRangingData &rangingData) {
  if (rangingData.measureType() != (uint8_t)uwb::MeasurementType::DL_TDOA) {
    return;
  }

  const RangingMesrDlTdoas dltdoa = rangingData.dlTdoaMeasure();
  uint8_t num_measurements = rangingData.available();

  Serial.println("\n========================================");
  Serial.print("Received ");
  Serial.print(num_measurements);
  Serial.println(" DL-TDoA measurements");
  Serial.println("========================================");

  // Process each measurement
  for (int i = 0; i < num_measurements; i++) {
    // Find which anchor this measurement is from
    uint8_t anchor_idx = 255;
    for (uint8_t j = 0; j < NUM_ANCHORS; j++) {
      if (dltdoa[i].peer_addr[0] == ANCHORS[j].mac[0] && 
          dltdoa[i].peer_addr[1] == ANCHORS[j].mac[1]) {
        anchor_idx = j;
        break;
      }
    }

    Serial.print("\n--- Measurement ");
    Serial.print(i + 1);
    Serial.println(" ---");
    
    if (anchor_idx != 255) {
      Serial.print("Anchor: ");
      Serial.print(anchor_idx + 1);
      Serial.print(" (MAC: 0x");
      Serial.print(ANCHORS[anchor_idx].mac[0], HEX);
      Serial.print(ANCHORS[anchor_idx].mac[1], HEX);
      Serial.print(")");
      Serial.print(" at position (");
      Serial.print(ANCHORS[anchor_idx].x, 1);
      Serial.print(", ");
      Serial.print(ANCHORS[anchor_idx].y, 1);
      Serial.println(") m");
    } else {
      Serial.print("Unknown Anchor (MAC: 0x");
      Serial.print(dltdoa[i].peer_addr[0], HEX);
      Serial.print(dltdoa[i].peer_addr[1], HEX);
      Serial.println(")");
    }

    // Display timestamps
    Serial.print("RX Timestamp: ");
    Serial.print(dltdoa[i].rx_timestamp);
    Serial.print(" (");
    Serial.print((double)dltdoa[i].rx_timestamp * TIMESTAMP_UNIT_SEC * 1e9, 3);
    Serial.println(" ns)");
    
    Serial.print("TX Timestamp: ");
    Serial.print(dltdoa[i].tx_timestamp);
    Serial.print(" (");
    Serial.print((double)dltdoa[i].tx_timestamp * TIMESTAMP_UNIT_SEC * 1e9, 3);
    Serial.println(" ns)");

    // Calculate time-of-flight estimate
    int64_t tof_ticks = (int64_t)dltdoa[i].rx_timestamp - (int64_t)dltdoa[i].tx_timestamp;
    double tof_sec = tof_ticks * TIMESTAMP_UNIT_SEC;
    float distance_estimate = tof_sec * SPEED_OF_LIGHT;
    
    Serial.print("Time-of-Flight: ");
    Serial.print(tof_sec * 1e9, 3);
    Serial.print(" ns");
    Serial.print(" (Distance estimate: ");
    Serial.print(distance_estimate, 3);
    Serial.println(" m)");

    // Display Angle of Arrival (AoA)
    float azimuth_deg = (float)dltdoa[i].aoa_azimuth / 2048.0 * 180.0;
    float elevation_deg = (float)dltdoa[i].aoa_elevation / 2048.0 * 180.0;
    
    Serial.print("AoA Azimuth: ");
    Serial.print(azimuth_deg, 2);
    Serial.print("°");
    Serial.print(" (FOM: ");
    Serial.print(dltdoa[i].aoa_azimuth_fom);
    Serial.println(")");
    
    Serial.print("AoA Elevation: ");
    Serial.print(elevation_deg, 2);
    Serial.println("°");

    // Display anchor-to-anchor ToF
    if (dltdoa[i].initiator_responder_tof > 0) {
      double anchor_tof_sec = dltdoa[i].initiator_responder_tof / 499.2e6;
      float anchor_distance = anchor_tof_sec * SPEED_OF_LIGHT;
      Serial.print("Anchor-to-Anchor ToF: ");
      Serial.print(anchor_tof_sec * 1e9, 3);
      Serial.print(" ns");
      Serial.print(" (Distance: ");
      Serial.print(anchor_distance, 3);
      Serial.println(" m)");
    }

    // Display status
    Serial.print("Status: ");
    if (dltdoa[i].status == 0) {
      Serial.println("Valid");
    } else {
      Serial.print("Error (code: ");
      Serial.print(dltdoa[i].status);
      Serial.println(")");
    }

    // Display block and round indices
    Serial.print("Block Index: ");
    Serial.print(dltdoa[i].block_index);
    Serial.print(", Round Index: ");
    Serial.println(dltdoa[i].round_index);
  }

  Serial.println("========================================\n");
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) {
    delay(10);
  }

#if defined(ARDUINO_PORTENTA_C33)
  pinMode(LEDB, OUTPUT);
  digitalWrite(LEDB, LOW);
#endif

  Serial.println("========================================");
  Serial.println("DL-TDoA Test Tag");
  Serial.println("========================================");
  Serial.print("Tag MAC Address: 0x");
  Serial.print(TAG_MAC[0], HEX);
  Serial.print(TAG_MAC[1], HEX);
  Serial.println();
  Serial.print("Session ID: 0x");
  Serial.println(SESSION_ID, HEX);
  Serial.println();
  Serial.println("Known Anchor Positions:");
  for (uint8_t i = 0; i < NUM_ANCHORS; i++) {
    Serial.print("  Anchor ");
    Serial.print(i + 1);
    Serial.print(": (");
    Serial.print(ANCHORS[i].x, 1);
    Serial.print(", ");
    Serial.print(ANCHORS[i].y, 1);
    Serial.print(") meters, MAC: 0x");
    Serial.print(ANCHORS[i].mac[0], HEX);
    Serial.print(ANCHORS[i].mac[1], HEX);
    Serial.println();
  }
  Serial.println("========================================");
  Serial.println("Note: This is a test setup that displays");
  Serial.println("measurements without full position calculation.");
  Serial.println("========================================\n");

  // Register ranging callback
  UWB.registerRangingCallback(rangingHandler);

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
  UWBMacAddress srcAddr(UWBMacAddress::Size::SHORT, TAG_MAC);

  // Create DL-TDoA Tag session
  // The tag listens to the ranging rounds between anchors
  UWBDltdoaTag tag(SESSION_ID, srcAddr, roundIndexList, sizeof(roundIndexList));

  // Add session to session manager
  UWBSessionManager.addSession(tag);

  // Initialize the session
  uwb::Status status = tag.init();
  if (status != uwb::Status::SUCCESS) {
    Serial.print("Session initialization failed with status: ");
    Serial.println((int)status);
    while (1) delay(1000);
  }
  Serial.println("Session initialized successfully");

  // Start ranging
  status = tag.start();
  if (status != uwb::Status::SUCCESS) {
    Serial.print("Failed to start ranging with status: ");
    Serial.println((int)status);
    while (1) delay(1000);
  }
  Serial.println("Tag is listening for anchor ranging sessions...");
  Serial.println("Waiting for measurements...\n");
}

void loop() {
#if defined(ARDUINO_PORTENTA_C33)
  digitalWrite(LEDB, !digitalRead(LEDB));
#endif
  delay(1000);
}

