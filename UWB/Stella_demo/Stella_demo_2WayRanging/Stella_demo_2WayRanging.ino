/**
  Two-Way Ranging Controller Example for Arduino Stella
  Name: stella_uwb_twr_controller.ino
  Purpose: This sketch configures the Arduino Stella as a Controller (Initiator)
  for Two-Way Ranging with a Portenta UWB Shield configured as Controlee.
  The LED provides visual feedback based on measured distance.
  
  @author Arduino Product Experience Team
  @version 1.0 15/04/25
*/

// Include required UWB library
#include <StellaUWB.h>

// Pin definitions
#define LED_PIN p37  // Stella's built-in LED for status indication

// Distance and timing parameters
#define MAX_DISTANCE 300     // Maximum distance to consider (cm)
#define MIN_BLINK_TIME 50    // Fastest blink rate (ms)
#define MAX_BLINK_TIME 1000  // Slowest blink rate (ms)
#define TIMEOUT_MS 2000      // Connection timeout (ms)

// System state variables
unsigned long lastBlink = 0;
unsigned long lastMeasurement = 0;
bool ledState = false;
int currentBlinkInterval = MAX_BLINK_TIME;
long lastDistance = MAX_DISTANCE;

/**
  Processes ranging data received from UWB communication.
  Updates LED feedback based on measured distance.
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
        // Update timing and distance tracking
        lastMeasurement = millis();
        lastDistance = twr[j].distance;

        // Calculate blink interval based on distance
        // Closer distance = faster blink
        if (lastDistance > MAX_DISTANCE) {
          currentBlinkInterval = MAX_BLINK_TIME;
        } else {
          // Map distance to blink interval
          currentBlinkInterval = map(lastDistance,
                                    0, MAX_DISTANCE,
                                    MIN_BLINK_TIME, MAX_BLINK_TIME);
        }

        // Display the distance measurement in centimeters
        Serial.print("- Distance (cm): ");
        Serial.println(lastDistance);
      }
    }
  }
}

void setup() {
  // Initialize serial communication at 115200 bits per second
  Serial.begin(115200);

  // Configure LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // Start with LED off

  Serial.println("- Arduino Stella - Two-Way Ranging Controller started...");
  
  // Define MAC addresses for this device and the target
  // This device (Controller) has address 0x2222
  // Target device (Controlee) has address 0x1111
  uint8_t devAddr[] = {0x22, 0x22};
  uint8_t destination[] = {0x11, 0x11};
  UWBMacAddress srcAddr(UWBMacAddress::Size::SHORT, devAddr);
  UWBMacAddress dstAddr(UWBMacAddress::Size::SHORT, destination);

  // Register the callback and start UWB
  UWB.registerRangingCallback(rangingHandler);
  UWB.begin();
  
  Serial.println("- Starting UWB...");
  
  // Wait until UWB stack is initialized
  while (UWB.state() != 0) {
    delay(10);
  }

  // Setup and start the UWB session using simplified UWBTracker
  Serial.println("- Starting session...");
  UWBTracker myTracker(0x11223344, srcAddr, dstAddr);
  UWBSessionManager.addSession(myTracker);
  myTracker.init();
  myTracker.start();

  // Signal initialization complete with triple LED flash
  Serial.println("- Initialization complete!");
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, LOW);   // LED ON
    delay(100);
    digitalWrite(LED_PIN, HIGH);  // LED OFF
    delay(100);
  }
}

void loop() {
  unsigned long currentTime = millis();

  // Handle LED feedback based on connection status and distance
  if (currentTime - lastMeasurement > TIMEOUT_MS) {
    // No connection detected - rapid blink warning
    if (currentTime - lastBlink >= 100) {
      lastBlink = currentTime;
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? LOW : HIGH);
    }
  } else {
    // Normal operation - distance-based blink rate
    if (currentTime - lastBlink >= currentBlinkInterval) {
      lastBlink = currentTime;
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? LOW : HIGH);
    }
  }

  // Small delay to prevent CPU overload
  delay(10);
}