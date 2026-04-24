// linearActuator_ai.ino
// Test script for a 12 V linear actuator driven by a Pololu MP6550.
// IN1 and IN2 are connected to Portenta C33 digital pins 8 and 9.
//
// Serial command interface:
//   'e' or 'E'  -> extend for 1 second
//   'r' or 'R'  -> retract for 1 second
//   's' or 'S'  -> stop immediately

const int RetractPin = 6;
const int ExtentPin = 7;
const int PumpPin = 1; // orange
const int SolenoidPin = 2; // green
float CleaningStage = 1; 


// Duration (ms) to run the actuator per command
float ActuatorDuration = 4000;
float SprayInterval = 1000;
float CurrentTime = 0;

static bool sprayOutputsActive() {
  return CleaningStage == 0;
}

void stopActuator() {
  // Brake / coast: both LOW (check datasheet; LOW/LOW is usually brake)
  digitalWrite(ExtentPin, LOW);
  digitalWrite(RetractPin, LOW);
}

void applySprayOutputs() {
  if (sprayOutputsActive()) {
    digitalWrite(SolenoidPin, LOW);
    if (CurrentTime == 0) {
      CurrentTime = millis();
      digitalWrite(PumpPin, HIGH);
    } else if (millis() >= CurrentTime + SprayInterval) {
      if (digitalRead(PumpPin) == HIGH){
        digitalWrite(PumpPin, LOW);
        CurrentTime = millis();
      } else {
        digitalWrite(PumpPin, HIGH);
        CurrentTime = millis();
      }
    }
    
  } else {
    digitalWrite(PumpPin, LOW);
    digitalWrite(SolenoidPin, HIGH);
  }
}

void extendActuator() {
  // One direction: IN1 HIGH, IN2 LOW
  digitalWrite(ExtentPin, HIGH);
  digitalWrite(RetractPin, LOW);
  delay(ActuatorDuration);
  digitalWrite(ExtentPin, LOW);
}

void retractActuator() {
  // Opposite direction: IN1 LOW, IN2 HIGH
  digitalWrite(ExtentPin, LOW);
  digitalWrite(RetractPin, HIGH);
  delay(ActuatorDuration);
  digitalWrite(RetractPin, LOW);
}

void handleCommand(char cmd) {
  if (cmd == 'e' || cmd == 'E') {
    Serial.println("Extending for 4 second...");
    extendActuator();
    delay(ActuatorDuration);
    stopActuator();
    Serial.println("Stopped.");
  } else if (cmd == 'r' || cmd == 'R') {
    Serial.println("Retracting for 4 second...");
    retractActuator();
    delay(ActuatorDuration);
    stopActuator();
    Serial.println("Stopped.");
  } else if (cmd == 'p' || cmd == 'P') {
    CleaningStage = 0;
    Serial.println("Pump command received.");
  } else if (cmd == 's' || cmd == 'S') {
    Serial.println("Stop command received.");
    stopActuator();
    CleaningStage = 1;
    applySprayOutputs();
  } else if (cmd != '\n' && cmd != '\r') {
    Serial.print("Unknown command: ");
    Serial.println(cmd);
  }
}

void setup() {
  pinMode(RetractPin, OUTPUT);
  pinMode(ExtentPin, OUTPUT);
  pinMode(PumpPin, OUTPUT);
  pinMode(SolenoidPin, OUTPUT);

  stopActuator();

  Serial.begin(9600);
  while (!Serial) {
    // Wait for USB serial on Portenta
  }

  Serial.println("Commands:");
  Serial.println("  'e' = extend for 4 second");
  Serial.println("  'r' = retract for 4 second");
  Serial.println("  's' = stop immediately");
  Serial.println("  'p' = start pump");
}


void loop() {
  // Check for incoming serial commands
  while (Serial.available() > 0) {
    char cmd = Serial.read();
    handleCommand(cmd);
  }
  applySprayOutputs();
}

