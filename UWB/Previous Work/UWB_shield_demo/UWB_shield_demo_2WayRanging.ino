/**
  Two-Way Ranging Controlee Example for Portenta UWB Shield
  Name: portenta_uwb_twr_controlee.ino
  Purpose: This sketch configures the Portenta UWB Shield as a Controlee (Responder)
  for Two-Way Ranging with an Arduino Stella configured as Controller.
  Includes distance visualization and moving average calculation.
  
  @author Arduino Product Experience Team
  @version 1.0 15/04/25
*/

// Include required UWB library
#include <PortentaUWBShield.h>

// Moving average configuration
#define SAMPLES 10                // Number of samples for moving average
long distances[SAMPLES] = {0};    // Circular buffer for distance measurements
int sample_index = 0;              // Current position in circular buffer

// LED and status configuration
#define NEARBY_THRESHOLD 300       // Distance threshold for green LED (cm)
#define CONNECTION_TIMEOUT 2000    // Time before considering tag lost (ms)
#define LED_BLINK_INTERVAL 500     // Red LED blink interval (ms)

// System state variables
unsigned long lastMeasurement = 0;
unsigned long lastLedBlink = 0;
bool ledState = false;

/**
  Processes ranging data received from UWB communication.
  Calculates moving average and provides visual feedback.
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
        // Update connection tracking
        lastMeasurement = millis();

        // Store new distance measurement in circular buffer
        distances[sample_index] = twr[j].distance;

        // Calculate moving average
        long avg = 0;
        for (int i = 0; i < SAMPLES; i++) {
          avg += distances[i];
        }
        avg = avg / SAMPLES;

        // Update distance indicator LED (Green LED)
        // LED ON when tag is nearby, OFF when far away
        digitalWrite(LEDG, (twr[j].distance <= NEARBY_THRESHOLD) ? LOW : HIGH);

        // Output formatted data for Serial Plotter
        Serial.print("Distance(cm):");
        Serial.print(twr[j].distance);
        Serial.print(",");
        Serial.print("Average (cm):");
        Serial.println(avg);

        // Update circular buffer index
        sample_index = (sample_index + 1) % SAMPLES;
      }
    }
  }
}

void setup() {
  // Initialize serial communication at 115200 bits per second
  Serial.begin(115200);
  
  #if defined(ARDUINO_PORTENTA_C33)
    // Initialize RGB LEDs (only Portenta C33 has RGB LED)
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    digitalWrite(LEDR, LOW);   // Red ON during initialization
    digitalWrite(LEDG, HIGH);  // Green OFF
    digitalWrite(LEDB, HIGH);  // Blue OFF
  #endif

  Serial.println("- Portenta UWB Shield - Two-Way Ranging Controlee started...");

  // Define MAC addresses for this device and the target
  // This device (Controlee) has address 0x1111
  // Target device (Controller) has address 0x2222
  uint8_t devAddr[] = {0x1111, 0x1111};
  uint8_t destination[] = {0x2222, 0x2222};
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

  // Setup and start the UWB session using simplified UWBRangingControlee
  Serial.println("- Starting session...");
  UWBRangingControlee myControlee(0x11223344, srcAddr, dstAddr);
  UWBSessionManager.addSession(myControlee);
  myControlee.init();
  myControlee.start();

  // Signal initialization complete
  Serial.println("- Initialization complete!");
  
  #if defined(ARDUINO_PORTENTA_C33)
    digitalWrite(LEDR, HIGH);  // Red OFF when initialized
  #endif
}

void loop() {
  unsigned long currentTime = millis();

  #if defined(ARDUINO_PORTENTA_C33)
    // Update connection status LED (Blue LED)
    // LED ON when no connection, OFF when connected
    digitalWrite(LEDB, (currentTime - lastMeasurement > CONNECTION_TIMEOUT) ? LOW : HIGH);

    // Blink red LED to show system is running
    if (currentTime - lastLedBlink >= LED_BLINK_INTERVAL) {
      lastLedBlink = currentTime;
      ledState = !ledState;
      digitalWrite(LEDR, ledState ? HIGH : LOW);
    }
  #else
    // For boards without RGB LED, print heartbeat
    if (currentTime - lastLedBlink >= LED_BLINK_INTERVAL) {
      lastLedBlink = currentTime;
      Serial.println("- System running...");
    }
  #endif

  // Small delay to prevent CPU overload
  delay(10);
}