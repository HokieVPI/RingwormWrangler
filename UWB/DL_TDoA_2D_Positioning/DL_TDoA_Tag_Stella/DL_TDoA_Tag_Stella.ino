/**
 * DL-TDoA Tag with 2D Position Calculation for Arduino Stella
 * 
 * This script configures an Arduino Stella board as a DL-TDoA Tag.
 * The tag listens to ranging sessions between anchors and calculates its own
 * position using Time Difference of Arrival (TDoA) and displays Angle of Arrival (AoA).
 * 
 * Features:
 * - Calculates 2D position (x, y) using trilateration
 * - Displays Angle of Arrival (AoA) from each anchor
 * - Uses known anchor positions for position calculation
 * 
 * Configuration:
 * - Session ID: 0x12345678 (must match anchors)
 * - Channel: 9
 * 
 * Libraries Required:
 * - StellaUWB-main
 * - ArduinoBLE-master (if BLE is needed for configuration)
 */

#include <StellaUWB.h>
#include <math.h>

// Pin definitions
#define LED_PIN p37  // Stella's built-in LED for status indication

// Tag configuration
const uint32_t SESSION_ID = 0x12345678;
const uint8_t TAG_MAC[] = {0xDD, 0xDD};

// Known anchor positions (in meters)
// These must match the positions configured in the anchor scripts
// NOTE: Anchors use centimeters (int32_t) but tag uses meters (float) for calculations
// Ensure values match: e.g., anchor at 220 cm = tag at 2.2 m
struct AnchorPosition {
  float x;  // X position in meters
  float y;  // Y position in meters
  uint8_t mac[2];  // MAC address
};

// Define anchor positions (must match anchor configurations)
// Values are in meters (anchors store them in centimeters: multiply by 100)
const AnchorPosition ANCHORS[] = {
  {0.0, 0.0, {0xAA, 0xAA}},      // Anchor 1 (Initiator): 0 cm = 0.0 m
  {2.2, 0, {0xBB, 0xBB}},        // Anchor 2 (Responder 1): 220 cm = 2.2 m
  {2.2, 2.2, {0xCC, 0xCC}}       // Anchor 3 (Responder 2): 220 cm = 2.2 m
};

const uint8_t NUM_ANCHORS = 3;

// Speed of light in meters per second
const float SPEED_OF_LIGHT = 299792458.0;

// Timestamp unit: 2^(-7) of 499.2 MHz chipping period ≈ 15.65 ps
const double TIMESTAMP_UNIT_SEC = 15.65e-12;

// Measurement storage
struct Measurement {
  uint64_t rx_timestamp;      // Reception timestamp at tag
  uint64_t tx_timestamp;      // Transmission timestamp from anchor
  int16_t aoa_azimuth;        // Angle of Arrival azimuth
  int16_t aoa_elevation;      // Angle of Arrival elevation
  uint8_t aoa_azimuth_fom;    // Figure of Merit for azimuth
  uint8_t anchor_mac[2];      // Anchor MAC address
  bool valid;                 // Is this measurement valid?
  uint32_t block_index;       // Block index
  uint8_t round_index;        // Round index
  uint16_t initiator_responder_tof; // Time-of-flight between anchors
};

Measurement measurements[NUM_ANCHORS];
uint8_t measurement_count = 0;

// Position calculation result
struct Position {
  float x;
  float y;
  bool valid;
};

Position current_position = {0.0, 0.0, false};

// Ranging data handler
void rangingHandler(UWBRangingData &rangingData) {
  if (rangingData.measureType() != (uint8_t)uwb::MeasurementType::DL_TDOA) {
    return;
  }

  uint8_t num_measurements = rangingData.available();
  Serial.print("Received ");
  Serial.print(num_measurements);
  Serial.println(" DL-TDoA measurements");

  // Reset measurement count
  measurement_count = 0;

  // DL-TDoA measurement access is now enabled in the StellaUWB library
  const RangingMesrDlTdoas dltdoa = rangingData.dlTdoaMeasure();
  
  // Process each measurement
  // Note: dltdoa is a pointer to the array, so we access it as dltdoa[i]
  for (int i = 0; i < num_measurements && measurement_count < NUM_ANCHORS; i++) {
    // Find which anchor this measurement is from
    // Check MAC address (first 2 bytes for short address)
    uint8_t anchor_idx = 255;
    for (uint8_t j = 0; j < NUM_ANCHORS; j++) {
      if (dltdoa[i].peer_addr[0] == ANCHORS[j].mac[0] && 
          dltdoa[i].peer_addr[1] == ANCHORS[j].mac[1]) {
        anchor_idx = j;
        break;
      }
    }

    if (anchor_idx == 255) {
      Serial.print("Unknown anchor MAC: ");
      Serial.print(dltdoa[i].peer_addr[0], HEX);
      Serial.print(":");
      Serial.println(dltdoa[i].peer_addr[1], HEX);
      continue;
    }

    // Store measurement
    measurements[measurement_count].rx_timestamp = dltdoa[i].rx_timestamp;
    measurements[measurement_count].tx_timestamp = dltdoa[i].tx_timestamp;
    measurements[measurement_count].aoa_azimuth = dltdoa[i].aoa_azimuth;
    measurements[measurement_count].aoa_elevation = dltdoa[i].aoa_elevation;
    measurements[measurement_count].aoa_azimuth_fom = dltdoa[i].aoa_azimuth_fom;
    measurements[measurement_count].anchor_mac[0] = dltdoa[i].peer_addr[0];
    measurements[measurement_count].anchor_mac[1] = dltdoa[i].peer_addr[1];
    measurements[measurement_count].block_index = dltdoa[i].block_index;
    measurements[measurement_count].round_index = dltdoa[i].round_index;
    measurements[measurement_count].initiator_responder_tof = dltdoa[i].initiator_responder_tof;
    measurements[measurement_count].valid = (dltdoa[i].status == 0);

    measurement_count++;

    // Display AoA information
    if (measurements[measurement_count - 1].valid) {
      Serial.print("Anchor ");
      Serial.print(anchor_idx + 1);
      Serial.print(" (");
      Serial.print(ANCHORS[anchor_idx].x);
      Serial.print(", ");
      Serial.print(ANCHORS[anchor_idx].y);
      Serial.print("): ");
      
      // Convert AoA from Q-format to degrees
      float azimuth_deg = (float)dltdoa[i].aoa_azimuth / 2048.0 * 180.0;
      float elevation_deg = (float)dltdoa[i].aoa_elevation / 2048.0 * 180.0;
      
      Serial.print("AoA Azimuth: ");
      Serial.print(azimuth_deg, 2);
      Serial.print("°, Elevation: ");
      Serial.print(elevation_deg, 2);
      Serial.print("°, FOM: ");
      Serial.println(dltdoa[i].aoa_azimuth_fom);
    }
  }

  // Calculate position if we have at least 3 measurements
  if (measurement_count >= 3) {
    calculatePosition();
  } else {
    Serial.print("Need at least 3 measurements, got ");
    Serial.println(measurement_count);
  }
}

// Calculate 2D position using TDoA trilateration
void calculatePosition() {
  // For DL-TDoA, we use the time differences of arrival
  // The tag receives signals from anchors at different times
  // We calculate the time differences and convert to distance differences
  
  // Find the earliest reception time (reference)
  uint64_t earliest_rx = measurements[0].rx_timestamp;
  for (uint8_t i = 1; i < measurement_count; i++) {
    if (measurements[i].rx_timestamp < earliest_rx) {
      earliest_rx = measurements[i].rx_timestamp;
    }
  }

  // Calculate time differences (TDoA)
  float distances[NUM_ANCHORS];
  
  for (uint8_t i = 0; i < measurement_count; i++) {
    if (!measurements[i].valid) continue;
    
    // Calculate time difference in seconds
    int64_t time_diff_ticks = (int64_t)measurements[i].rx_timestamp - (int64_t)earliest_rx;
    double time_diff_sec = time_diff_ticks * TIMESTAMP_UNIT_SEC;
    
    // For DL-TDoA, we need to account for the anchor-to-anchor ranging
    // The actual distance is calculated from the time-of-flight
    // We use the tx_timestamp and rx_timestamp to estimate distance
    int64_t tof_ticks = (int64_t)measurements[i].rx_timestamp - (int64_t)measurements[i].tx_timestamp;
    double tof_sec = tof_ticks * TIMESTAMP_UNIT_SEC;
    distances[i] = tof_sec * SPEED_OF_LIGHT;  // One-way distance estimate
  }

  // Use trilateration to calculate position
  // For 2D positioning with 3 anchors, we solve:
  // (x - x1)^2 + (y - y1)^2 = d1^2
  // (x - x2)^2 + (y - y2)^2 = d2^2
  // (x - x3)^2 + (y - y3)^2 = d3^2
  
  float x1 = ANCHORS[0].x, y1 = ANCHORS[0].y, d1 = distances[0];
  float x2 = ANCHORS[1].x, y2 = ANCHORS[1].y, d2 = distances[1];
  float x3 = ANCHORS[2].x, y3 = ANCHORS[2].y, d3 = distances[2];

  // Trilateration using linear algebra
  // Subtract first equation from others to eliminate quadratic terms
  float A = 2 * (x2 - x1);
  float B = 2 * (y2 - y1);
  float C = d1*d1 - d2*d2 - x1*x1 + x2*x2 - y1*y1 + y2*y2;
  
  float D = 2 * (x3 - x1);
  float E = 2 * (y3 - y1);
  float F = d1*d1 - d3*d3 - x1*x1 + x3*x3 - y1*y1 + y3*y3;

  // Solve: Ax + By = C and Dx + Ey = F
  float det = A * E - B * D;
  
  if (abs(det) < 1e-6) {
    Serial.println("Error: Anchors are collinear, cannot calculate position");
    current_position.valid = false;
    return;
  }

  float x = (C * E - B * F) / det;
  float y = (A * F - C * D) / det;

  current_position.x = x;
  current_position.y = y;
  current_position.valid = true;

  // Display results
  Serial.println("\n=== Position Calculation ===");
  Serial.print("Calculated Position: (");
  Serial.print(x, 3);
  Serial.print(", ");
  Serial.print(y, 3);
  Serial.println(") meters");
  
  // Display distances to each anchor
  Serial.println("Distances to anchors:");
  for (uint8_t i = 0; i < measurement_count; i++) {
    if (measurements[i].valid) {
      float dx = x - ANCHORS[i].x;
      float dy = y - ANCHORS[i].y;
      float calc_dist = sqrt(dx*dx + dy*dy);
      Serial.print("  Anchor ");
      Serial.print(i + 1);
      Serial.print(": measured=");
      Serial.print(distances[i], 3);
      Serial.print("m, calculated=");
      Serial.print(calc_dist, 3);
      Serial.println("m");
    }
  }
  
  // Display AoA summary
  Serial.println("Angle of Arrival (AoA):");
  for (uint8_t i = 0; i < measurement_count; i++) {
    if (measurements[i].valid) {
      float azimuth_deg = (float)measurements[i].aoa_azimuth / 2048.0 * 180.0;
      float elevation_deg = (float)measurements[i].aoa_elevation / 2048.0 * 180.0;
      
      // Calculate expected angle from position
      float dx = current_position.x - ANCHORS[i].x;
      float dy = current_position.y - ANCHORS[i].y;
      float expected_angle = atan2(dy, dx) * 180.0 / M_PI;
      
      Serial.print("  Anchor ");
      Serial.print(i + 1);
      Serial.print(": Azimuth=");
      Serial.print(azimuth_deg, 1);
      Serial.print("° (expected=");
      Serial.print(expected_angle, 1);
      Serial.print("°), Elevation=");
      Serial.print(elevation_deg, 1);
      Serial.println("°");
    }
  }
  Serial.println("===========================\n");
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) {
    delay(10);
  }

  // Configure LED pin for Stella
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // Start with LED off (HIGH = off on Stella)

  Serial.println("DL-TDoA Tag with 2D Position Calculation (Arduino Stella)");
  Serial.println("==========================================================");
  Serial.print("Known Anchor Positions:\n");
  for (uint8_t i = 0; i < NUM_ANCHORS; i++) {
    Serial.print("  Anchor ");
    Serial.print(i + 1);
    Serial.print(": (");
    Serial.print(ANCHORS[i].x);
    Serial.print(", ");
    Serial.print(ANCHORS[i].y);
    Serial.print(") meters, MAC: ");
    Serial.print(ANCHORS[i].mac[0], HEX);
    Serial.print(":");
    Serial.println(ANCHORS[i].mac[1], HEX);
  }
  Serial.println();

  // Register ranging callback
  UWB.registerRangingCallback(rangingHandler);

  // Initialize UWB stack
  UWB.begin(Serial, uwb::LogLevel::UWB_INFO_LEVEL);
  Serial.println("UWB stack initialized");

  // Wait for UWB stack to be ready
  while (UWB.state() != 0) {
    delay(10);
  }
  Serial.println("UWB stack ready");

  // Create source MAC address
  UWBMacAddress srcAddr(UWBMacAddress::Size::SHORT, TAG_MAC);

  // Create a UWBSession and configure it for DL-TDoA Tag mode
  // Since StellaUWB doesn't have a dedicated DL-TDoA tag class,
  // we need to manually configure a session
  UWBSession tagSession;
  tagSession.sessionID(SESSION_ID);
  tagSession.sessionType(uwb::SessionType::RANGING);
  
  // Configure ranging parameters for DL-TDoA Tag
  tagSession.rangingParams.deviceRole(uwb::DeviceRole::DL_TDOA_TAG);
  tagSession.rangingParams.deviceType(uwb::DeviceType::CONTROLEE);
  tagSession.rangingParams.multiNodeMode(uwb::MultiNodeMode::ONE_TO_MANY);
  tagSession.rangingParams.rangingRoundUsage(uwb::RangingMethod::DL_TDOA);
  tagSession.rangingParams.scheduledMode(uwb::ScheduledMode::TIME_SCHEDULED);
  tagSession.rangingParams.deviceMacAddr(srcAddr);
  
  // Configure app parameters
  tagSession.appParams.frameConfig(uwb::RfFrameConfig::Sfd_Sts);
  tagSession.appParams.stsConfig(uwb::StsConfig::StaticSts);
  tagSession.appParams.channel(9);
  tagSession.appParams.preambleCodeIndex(10);
  tagSession.appParams.sfdId(0);
  tagSession.appParams.addOrUpdateParam(buildScalar(uwb::AppConfigId::SessionInfoNtf, 1));
  tagSession.appParams.rangingDuration(200);
  tagSession.appParams.slotPerRR(10);
  tagSession.appParams.slotDuration(1200);
  tagSession.appParams.noOfControlees(1);
  
  // Add session to session manager
  UWBSessionManager.addSession(tagSession);
  
  // Initialize the session
  uwb::Status status = tagSession.init();
  if (status != uwb::Status::SUCCESS) {
    Serial.print("Session initialization failed with status: ");
    Serial.println((int)status);
    while (1) delay(1000);
  }
  Serial.println("Session initialized successfully");

  // Enable ranging data notifications
  tagSession.enableRangingDataNtf(1);

  // Start ranging
  status = tagSession.start();
  if (status != uwb::Status::SUCCESS) {
    Serial.print("Failed to start ranging with status: ");
    Serial.println((int)status);
    while (1) delay(1000);
  }
  Serial.println("Tag is listening for anchor ranging sessions...");
  Serial.println("Waiting for measurements...\n");
  Serial.println("DL-TDoA measurement access is enabled in the library.");
}

void loop() {
  // Toggle LED to indicate system is running
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  delay(1000);
  
  // Periodically display current position if valid
  if (current_position.valid) {
    Serial.print("Current Position: (");
    Serial.print(current_position.x, 3);
    Serial.print(", ");
    Serial.print(current_position.y, 3);
    Serial.println(") meters");
  }
}

