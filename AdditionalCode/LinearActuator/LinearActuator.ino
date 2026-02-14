const int servoExtend = 1
const int servoRetract = 2
bool mopStatus = false

void setup() {
  pinmode(servoExtend,OUTPUT)
  pinmode(servoRetract,OUTPUT)
  // put your setup code here, to run once:
}

void loop() {
  // extends the mop
  if(mopStatus == true) {
    digitalWrite(servoExtend, HIGH)
    digitalWrite(servoRetract, LOW)
  }
  // retracts the mop
  else if(mopStatus == false) {
    digitalWrite(servoExtend, LOW)
    digitalWrite(servoRetract, HIGH)
  }
  // default status
  else {
    digitalWrite(servoExtend, HIGH)
    digitalWrite(servoRetract, HIGH)
  }

  // put your main code here, to run repeatedly:

}
