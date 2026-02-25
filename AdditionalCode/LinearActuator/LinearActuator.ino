const int RPWM = 9;
const int LPWM = 10;
int speed = 0;
int mopStatus = 0;
int Right = LOW;
int Left = LOW;

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
    digitalWrite(RPWM, LOW);
    digitalWrite(LPWM, HIGH);
    delay(1000);
    mopStatus = 0; 
  }
  // retracts the mop
  else if (mopStatus == 0) {
    speed = 127;
    digitalWrite(RPWM, HIGH);
    digitalWrite(LPWM, LOW);
    delay(1000);
  }
  // default status (off)
  else {
    digitalWrite(RPWM, LOW);
    digitalWrite(LPWM, LOW);
    delay(1000);
  }
  Right = digitalRead(RPWM);
  Left = digitalRead(LPWM);
  Serial.print(Right);
  Serial.print(Left);
  Serial.print(mopStatus);

  delay(100);
}
