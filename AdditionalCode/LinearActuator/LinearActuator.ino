const int RPWM = 11;
const int LPWM = 12;
int speed = 0;
int mopStatus = 0;

void setup() {
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  Serial.println("Mop Status (1 = Extend / 0 = Retract, other = Off)");

  if (Serial.available() > 0) {
    int userInput = Serial.parseInt();

    if (userInput == 1) {
      mopStatus = 1;
    } else if (userInput == 0) {
      mopStatus = 0;
    } else {
      mopStatus = -1;
    }
  }

  // extends the mop
  if (mopStatus == 1) {
    speed = 127;
    analogWrite(RPWM, LOW);
    analogWrite(LPWM, HIGH);
  }
  // retracts the mop
  else if (mopStatus == 0) {
    speed = 127;
    analogWrite(RPWM, HIGH);
    analogWrite(LPWM, LOW);
  }
  // default status (off)
  else {
    analogWrite(RPWM, LOW);
    analogWrite(LPWM, LOW);
  }

  delay(100);
}
