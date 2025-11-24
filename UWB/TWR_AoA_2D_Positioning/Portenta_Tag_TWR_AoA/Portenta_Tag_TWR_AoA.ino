/**
  Two-Way Ranging Controller with AoA 2D Positioning for Portenta C33 UWB Shield
  Name: portenta_uwb_twr_aoa_2d_tag.ino
  Purpose: This sketch configures the Portenta C33 with UWB Shield as a Controller (Initiator/Tag)
  for Two-Way Ranging with 2 Portenta UWB Shield anchors configured as Controlees.
  Calculates 2D position using hybrid trilateration/triangulation approach.
  
  @author Modified for 2D positioning with Portenta C33
  @version 2.0
*/

// Include required UWB library
#include <PortentaUWBShield.h>
#include <math.h>

// Distance and timing parameters
#define MAX_DISTANCE 300     // Maximum distance to consider (cm)
#define TIMEOUT_MS 2000      // Connection timeout (ms)
#define NUM_ANCHORS 2        // Number of anchors

// Anchor configuration - USER CONFIGURABLE
struct AnchorConfig {
  float x;           // X position in meters
  float y;           // Y position in meters
  uint8_t mac[2];    // MAC address
};

// Define anchor positions (modify these to match your physical setup)
const AnchorConfig ANCHORS[NUM_ANCHORS] = {
  {0.0, 0.0, {0x11, 0x11}},    // Anchor 1: MAC 0x1111, position (x1, y1) in meters
  {2.0, 0.0, {0x33, 0x33}}     // Anchor 2: MAC 0x3333, position (x2, y2) in meters
};

// Measurement storage
struct Measurement {
  float distance_cm;      // Distance in centimeters
  float distance_m;      // Distance in meters
  float aoa_degrees;      // Angle of Arrival in degrees
  uint8_t anchor_mac[2]; // Anchor MAC address
  uint8_t anchor_idx;    // Anchor index (0 or 1)
  bool valid;            // Is this measurement valid?
  unsigned long timestamp; // Measurement timestamp
};

Measurement measurements[NUM_ANCHORS];
bool measurements_ready = false;

// Track which anchor we're currently receiving from
// Since we have two sessions, we'll use a simple round-robin approach
static uint8_t current_anchor_tracker = 0;

// Position calculation result
struct Position {
  float x;      // X position in meters
  float y;      // Y position in meters
  bool valid;   // Is position valid?
};

Position current_position = {0.0, 0.0, false};

// System state variables
unsigned long lastBlink = 0;
unsigned long lastMeasurement = 0;
bool ledState = false;

// PI constant for angle calculations
#ifndef PI
#define PI 3.14159265359
#endif

/**
  Processes ranging data received from UWB communication.
  Extracts distance and AoA, stores measurements from both anchors.
  @param rangingData Reference to UWB ranging data object.
*/
void rangingHandler(UWBRangingData &rangingData) {
  if (rangingData.measureType() == (uint8_t)uwb::MeasurementType::TWO_WAY) {
    // Get the TWR (Two-Way Ranging) measurements
    RangingMeasures twr = rangingData.twoWayRangingMeasure();

    // Loop through all available measurements
    for (int j = 0; j < rangingData.available(); j++) {
      // Only process valid measurements
      if (twr[j].status == 0 && twr[j].distance != 0xFFFF) {
        // Update timing
        lastMeasurement = millis();

        // Extract distance
        float distance_cm = (float)twr[j].distance;
        float distance_m = distance_cm / 100.0;

        // Extract AoA - Note: TWR measurements don't directly provide AoA
        // AoA is typically available in DL-TDoA mode, not TWR
        // For now, set to 0 and rely on trilateration only
        // If AoA becomes available, it would need to be extracted from the measurement structure
        float angle_degrees = 0.0;  // AoA not available in TWR mode
        
        // Identify anchor index
        // Since we have two separate sessions, measurements alternate
        // We'll use a simple approach: alternate between anchors
        uint8_t anchor_idx = current_anchor_tracker % NUM_ANCHORS;
        current_anchor_tracker = (current_anchor_tracker + 1) % NUM_ANCHORS;

        // Store measurement for this anchor
        measurements[anchor_idx].distance_cm = distance_cm;
        measurements[anchor_idx].distance_m = distance_m;
        measurements[anchor_idx].aoa_degrees = angle_degrees;
        measurements[anchor_idx].anchor_idx = anchor_idx;
        measurements[anchor_idx].anchor_mac[0] = ANCHORS[anchor_idx].mac[0];
        measurements[anchor_idx].anchor_mac[1] = ANCHORS[anchor_idx].mac[1];
        measurements[anchor_idx].valid = true;
        measurements[anchor_idx].timestamp = millis();

        // Check if we have measurements from both anchors
        bool both_valid = measurements[0].valid && measurements[1].valid;
        // Only calculate if measurements are recent (within 1000ms of each other)
        unsigned long time_diff = abs((long)measurements[0].timestamp - (long)measurements[1].timestamp);
        if (both_valid && time_diff < 1000) {  // Within 1 second
          measurements_ready = true;
        }

        // Display measurement
        Serial.print("Anchor ");
        Serial.print(anchor_idx + 1);
        Serial.print(" (MAC 0x");
        Serial.print(ANCHORS[anchor_idx].mac[0], HEX);
        Serial.print(ANCHORS[anchor_idx].mac[1], HEX);
        Serial.print("): Distance=");
        Serial.print(distance_cm, 1);
        Serial.print("cm, AoA=");
        Serial.print(angle_degrees, 1);
        Serial.println("°");
      }
    }
    
    // If we have measurements from both anchors, calculate position
    if (measurements_ready) {
      calculate2DPosition();
      measurements_ready = false; // Reset flag
    }
  }
}

/**
  Calculate 2D position using hybrid trilateration/triangulation approach
*/
void calculate2DPosition() {
  // Check if we have valid measurements from both anchors
  if (!measurements[0].valid || !measurements[1].valid) {
    Serial.println("Error: Missing measurements from one or both anchors");
    current_position.valid = false;
    return;
  }

  float d1 = measurements[0].distance_m;
  float d2 = measurements[1].distance_m;
  float theta1 = measurements[0].aoa_degrees;
  float theta2 = measurements[1].aoa_degrees;
  
  float x1 = ANCHORS[0].x;
  float y1 = ANCHORS[0].y;
  float x2 = ANCHORS[1].x;
  float y2 = ANCHORS[1].y;

  // Step 1: Trilateration using distances
  Position pos_trilat;
  calculateTrilateration(d1, d2, x1, y1, x2, y2, &pos_trilat);

  // Step 2: Triangulation using AoA
  Position pos_triang;
  calculateTriangulation(theta1, theta2, x1, y1, x2, y2, &pos_triang);

  // Step 3: Hybrid fusion (weighted combination)
  // Use equal weights for now (can be adjusted based on measurement quality)
  float w_trilat = 0.5;
  float w_triang = 0.5;
  
  // If triangulation failed, use only trilateration
  if (!pos_triang.valid) {
    w_trilat = 1.0;
    w_triang = 0.0;
  }
  
  // If trilateration failed, use only triangulation
  if (!pos_trilat.valid) {
    w_trilat = 0.0;
    w_triang = 1.0;
  }

  // Calculate final position
  if (pos_trilat.valid || pos_triang.valid) {
    current_position.x = w_trilat * pos_trilat.x + w_triang * pos_triang.x;
    current_position.y = w_trilat * pos_trilat.y + w_triang * pos_triang.y;
    current_position.valid = true;

    // Display results
    Serial.println("\n=== Position Calculation ===");
    Serial.print("Trilateration: (");
    Serial.print(pos_trilat.x, 3);
    Serial.print(", ");
    Serial.print(pos_trilat.y, 3);
    Serial.print(") ");
    Serial.println(pos_trilat.valid ? "valid" : "invalid");
    
    Serial.print("Triangulation: (");
    Serial.print(pos_triang.x, 3);
    Serial.print(", ");
    Serial.print(pos_triang.y, 3);
    Serial.print(") ");
    Serial.println(pos_triang.valid ? "valid" : "invalid");
    
    Serial.print("Final Position (Hybrid): (");
    Serial.print(current_position.x, 3);
    Serial.print(", ");
    Serial.print(current_position.y, 3);
    Serial.println(") meters");
    Serial.println("===========================\n");
  } else {
    current_position.valid = false;
    Serial.println("Error: Both trilateration and triangulation failed");
  }
}

/**
  Calculate position using trilateration (distance-based)
  Solves: (x-x1)² + (y-y1)² = d1² and (x-x2)² + (y-y2)² = d2²
*/
void calculateTrilateration(float d1, float d2, float x1, float y1, float x2, float y2, Position* result) {
  // Calculate distance between anchors
  float anchor_dist = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
  
  // Check if distances are valid (triangle inequality)
  if (d1 + d2 < anchor_dist || abs(d1 - d2) > anchor_dist) {
    result->valid = false;
    return;
  }

  // Solve system of equations using linear algebra
  // Expand: x² - 2x1*x + x1² + y² - 2y1*y + y1² = d1²
  //         x² - 2x2*x + x2² + y² - 2y2*y + y2² = d2²
  // Subtract: 2(x2-x1)x + 2(y2-y1)y = d1² - d2² - x1² + x2² - y1² + y2²
  
  float A = 2 * (x2 - x1);
  float B = 2 * (y2 - y1);
  float C = d1 * d1 - d2 * d2 - x1 * x1 + x2 * x2 - y1 * y1 + y2 * y2;

  // If anchors are vertically aligned (A = 0)
  if (abs(A) < 1e-6) {
    if (abs(B) < 1e-6) {
      // Anchors are at same position - invalid
      result->valid = false;
      return;
    }
    // Solve for y, then x
    float y = C / B;
    // Use first circle equation to find x
    float x_squared = d1 * d1 - (y - y1) * (y - y1);
    if (x_squared < 0) {
      result->valid = false;
      return;
    }
    float x = sqrt(x_squared) + x1;
    result->x = x;
    result->y = y;
    result->valid = true;
    return;
  }

  // If anchors are horizontally aligned (B = 0)
  if (abs(B) < 1e-6) {
    float x = C / A;
    // Use first circle equation to find y
    float y_squared = d1 * d1 - (x - x1) * (x - x1);
    if (y_squared < 0) {
      result->valid = false;
      return;
    }
    float y = sqrt(y_squared) + y1;
    result->x = x;
    result->y = y;
    result->valid = true;
    return;
  }

  // General case: solve for y in terms of x, then substitute
  // y = (C - A*x) / B
  // Substitute into first circle equation
  float a = 1 + (A * A) / (B * B);
  float b = -2 * x1 - 2 * (A / B) * ((C / B) - y1);
  float c = x1 * x1 + ((C / B) - y1) * ((C / B) - y1) - d1 * d1;

  float discriminant = b * b - 4 * a * c;
  if (discriminant < 0) {
    result->valid = false;
    return;
  }

  float x = (-b + sqrt(discriminant)) / (2 * a);
  float y = (C - A * x) / B;
  
  result->x = x;
  result->y = y;
  result->valid = true;
}

/**
  Calculate position using triangulation (AoA-based)
  Uses bearing lines from both anchors
*/
void calculateTriangulation(float theta1, float theta2, float x1, float y1, float x2, float y2, Position* result) {
  // Convert angles to radians
  float angle1_rad = theta1 * (PI / 180.0);
  float angle2_rad = theta2 * (PI / 180.0);

  // Calculate bearing lines from each anchor
  // Line 1: y - y1 = tan(angle1) * (x - x1)
  // Line 2: y - y2 = tan(angle2) * (x - x2)
  
  // Check for vertical lines (angle = 90° or 270°)
  bool vert1 = (abs(cos(angle1_rad)) < 1e-6);
  bool vert2 = (abs(cos(angle2_rad)) < 1e-6);

  if (vert1 && vert2) {
    // Both lines are vertical - parallel, no intersection
    result->valid = false;
    return;
  }

  if (vert1) {
    // First line is vertical: x = x1
    float x = x1;
    float m2 = tan(angle2_rad);
    float y = y2 + m2 * (x - x2);
    result->x = x;
    result->y = y;
    result->valid = true;
    return;
  }

  if (vert2) {
    // Second line is vertical: x = x2
    float x = x2;
    float m1 = tan(angle1_rad);
    float y = y1 + m1 * (x - x1);
    result->x = x;
    result->y = y;
    result->valid = true;
    return;
  }

  // General case: both lines have slopes
  float m1 = tan(angle1_rad);
  float m2 = tan(angle2_rad);

  // Check if lines are parallel
  if (abs(m1 - m2) < 1e-6) {
    result->valid = false;
    return;
  }

  // Solve intersection:
  // y = m1*(x - x1) + y1
  // y = m2*(x - x2) + y2
  // m1*(x - x1) + y1 = m2*(x - x2) + y2
  // m1*x - m1*x1 + y1 = m2*x - m2*x2 + y2
  // (m1 - m2)*x = m1*x1 - y1 - m2*x2 + y2
  // x = (m1*x1 - y1 - m2*x2 + y2) / (m1 - m2)
  
  float x = (m1 * x1 - y1 - m2 * x2 + y2) / (m1 - m2);
  float y = m1 * (x - x1) + y1;
  
  result->x = x;
  result->y = y;
  result->valid = true;
}

void setup() {
  // Initialize serial communication at 115200 bits per second
  Serial.begin(115200);
  while (!Serial && millis() < 5000) {
    delay(10);
  }

  #if defined(ARDUINO_PORTENTA_C33)
    // Initialize RGB LEDs (only Portenta C33 has RGB LED)
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    digitalWrite(LEDR, LOW);   // Red ON during initialization
    digitalWrite(LEDG, HIGH);  // Green OFF
    digitalWrite(LEDB, HIGH);  // Blue OFF
  #endif

  Serial.println("========================================");
  Serial.println("Portenta C33 - TWR AoA 2D Positioning Tag");
  Serial.println("========================================");
  Serial.print("Tag MAC: 0x2222\n");
  Serial.print("Anchor 1: MAC 0x");
  Serial.print(ANCHORS[0].mac[0], HEX);
  Serial.print(ANCHORS[0].mac[1], HEX);
  Serial.print(", Position (");
  Serial.print(ANCHORS[0].x, 2);
  Serial.print(", ");
  Serial.print(ANCHORS[0].y, 2);
  Serial.println(") m");
  Serial.print("Anchor 2: MAC 0x");
  Serial.print(ANCHORS[1].mac[0], HEX);
  Serial.print(ANCHORS[1].mac[1], HEX);
  Serial.print(", Position (");
  Serial.print(ANCHORS[1].x, 2);
  Serial.print(", ");
  Serial.print(ANCHORS[1].y, 2);
  Serial.println(") m");
  Serial.println("========================================\n");
  
  // Define MAC addresses for this device (tag)
  uint8_t devAddr[] = {0x22, 0x22};
  UWBMacAddress srcAddr(UWBMacAddress::Size::SHORT, devAddr);

  // Register the callback and start UWB
  UWB.registerRangingCallback(rangingHandler);
  UWB.begin();
  
  Serial.println("Starting UWB...");
  
  // Wait until UWB stack is initialized
  unsigned long startTime = millis();
  while (UWB.state() != 0 && (millis() - startTime) < 10000) {
    delay(10);
  }
  
  if (UWB.state() != 0) {
    Serial.println("ERROR: UWB stack failed to initialize!");
    #if defined(ARDUINO_PORTENTA_C33)
      digitalWrite(LEDR, LOW);  // Red ON for error
    #endif
    while (1) delay(1000);
  }

  // IMPORTANT: Manual UWBSession configuration doesn't support setting destination addresses
  // directly like UWBTracker/UWBRangingControlee do. For TWR Controller on Portenta,
  // we may need to use a workaround or the library may not fully support this use case.
  // 
  // Attempt: Use ONE_TO_MANY mode - the UWB stack may discover peers automatically
  // based on session ID, or we may need to configure anchors differently.
  Serial.println("Creating session for both anchors...");
  Serial.println("WARNING: Manual session config may not support destination addresses");
  Serial.println("If this fails, consider using Stella as tag or modifying anchor config\n");
  
  UWBSession tagSession;
  tagSession.sessionID(0x11223344);  // Same session ID as anchors
  tagSession.sessionType(uwb::SessionType::RANGING);
  
  // Configure ranging parameters for TWR Controller
  // Try ONE_TO_MANY mode - UWB stack may auto-discover peers with same session ID
  tagSession.rangingParams.deviceType(uwb::DeviceType::CONTROLLER);
  tagSession.rangingParams.multiNodeMode(uwb::MultiNodeMode::ONE_TO_MANY);
  tagSession.rangingParams.deviceMacAddr(srcAddr);
  
  // Configure app parameters (matching anchor configuration)
  tagSession.appParams.frameConfig(uwb::RfFrameConfig::Sfd_Sts);
  tagSession.appParams.stsConfig(uwb::StsConfig::StaticSts);
  tagSession.appParams.channel(9);
  tagSession.appParams.preambleCodeIndex(10);
  tagSession.appParams.sfdId(0);
  tagSession.appParams.addOrUpdateParam(buildScalar(uwb::AppConfigId::SessionInfoNtf, 1));
  tagSession.appParams.rangingDuration(200);
  tagSession.appParams.slotPerRR(10);
  tagSession.appParams.slotDuration(1200);
  tagSession.appParams.noOfControlees(2);  // Expect 2 anchors
  
  // Add session to session manager
  UWBSessionManager.addSession(tagSession);
  
  // Initialize the session
  uwb::Status status = tagSession.init();
  if (status != uwb::Status::SUCCESS) {
    Serial.print("Session initialization failed with status: ");
    Serial.println((int)status);
    Serial.println("\nERROR: Manual UWBSession may not support TWR Controller role");
    Serial.println("Consider: Use Arduino Stella as tag, or check library documentation");
    #if defined(ARDUINO_PORTENTA_C33)
      digitalWrite(LEDR, LOW);  // Red ON for error
    #endif
    while (1) delay(1000);
  } else {
    Serial.println("Session initialized successfully");
    
    // Enable ranging data notifications
    tagSession.enableRangingDataNtf(1);
    
    // Start ranging
    status = tagSession.start();
    if (status != uwb::Status::SUCCESS) {
      Serial.print("Session failed to start with status: ");
      Serial.println((int)status);
      Serial.println("\nPossible issues:");
      Serial.println("1. Destination addresses not configured (manual session limitation)");
      Serial.println("2. PortentaUWBShield may not support TWR Controller via manual session");
      Serial.println("3. Try using Arduino Stella as tag instead");
      #if defined(ARDUINO_PORTENTA_C33)
        digitalWrite(LEDR, LOW);  // Red ON for error
      #endif
    } else {
      Serial.println("Session started - attempting to range with anchors");
      Serial.println("Note: If no measurements received, manual session may not support destinations");
    }
  }

  // Signal initialization complete
  Serial.println("\nInitialization complete! Waiting for measurements...\n");
  #if defined(ARDUINO_PORTENTA_C33)
    // Triple flash with RGB LEDs
    for (int i = 0; i < 3; i++) {
      digitalWrite(LEDR, LOW);   // Red ON
      digitalWrite(LEDG, LOW);   // Green ON
      delay(100);
      digitalWrite(LEDR, HIGH);  // Red OFF
      digitalWrite(LEDG, HIGH); // Green OFF
      delay(100);
    }
    digitalWrite(LEDR, HIGH);  // Red OFF
  #endif
}

void loop() {
  unsigned long currentTime = millis();

  #if defined(ARDUINO_PORTENTA_C33)
    // Handle LED feedback based on connection status
    if (currentTime - lastMeasurement > TIMEOUT_MS) {
      // No connection detected - Blue LED ON (warning)
      digitalWrite(LEDB, LOW);  // Blue ON
    } else {
      // Connected - Blue LED OFF
      digitalWrite(LEDB, HIGH); // Blue OFF
    }

    // Blink green LED to show system is running
    if (currentTime - lastBlink >= 1000) {
      lastBlink = currentTime;
      ledState = !ledState;
      digitalWrite(LEDG, ledState ? LOW : HIGH);  // Green ON when ledState is true
    }
  #endif

  // Periodically display current position if valid
  static unsigned long lastPositionPrint = 0;
  if (current_position.valid && (currentTime - lastPositionPrint >= 5000)) {
    lastPositionPrint = currentTime;
    Serial.print("Current Position: (");
    Serial.print(current_position.x, 3);
    Serial.print(", ");
    Serial.print(current_position.y, 3);
    Serial.println(") meters");
  }

  // Small delay to prevent CPU overload
  delay(10);
}

