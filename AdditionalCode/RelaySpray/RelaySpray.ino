// Use Relay to Spray //
unsigned long previousMillis = 0;
const unsigned long interval = 2000;  // 1 second
bool relayState = false;

void setup() {
  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);
  Serial.begin(115200);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    relayState = !relayState;
    digitalWrite(8, relayState);

    Serial.print("Relay is now on");
    Serial.println(relayState);
  }


}