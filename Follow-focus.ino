#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

Encoder myEnc(2, 3);

static int fwd = 5;
static int bac = 6;
static int pwm = 9;
static int center = 515;
long oldPosition  = 0;
long oldTime = 0;
int thresh = 1 * 1000;
float rpm = 0;
int pwmV = 0;

// the setup routine runs once when you press reset:
void setup() {
  pinMode(fwd, OUTPUT);
  pinMode(bac, OUTPUT);
  pinMode(pwm, OUTPUT);
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  int sensorValue = analogRead(A0);
  int currentValue = analogRead(A9);
  long tgtSpeed = abs(sensorValue - center);
  if (tgtSpeed < 2) {
    tgtSpeed = 0;
  }
  tgtSpeed = map(tgtSpeed, 0, 512, 0, 24000);
  long newTime = micros();
  long deltaT = newTime - oldTime;
  long newPosition = myEnc.read();
  myEnc.write(0);
  oldTime = micros();
  // (change in position / encoder tics per rev) * micros in a second / change in time
  rpm = (abs(newPosition - oldPosition) / 12.0) * (60000.0 * 1000) / (deltaT * 1.0);
  oldPosition = 0; 
  float increment = map(abs(rpm - tgtSpeed), 0, 24000, 0, 50);

  if (rpm < tgtSpeed) {
    pwmV += increment;
  }
  else if (rpm > tgtSpeed) {
    pwmV -= increment;
  }
  //  int pwmV = abs(sensorValue - center)/2;
  pwmV = constrain(pwmV, 0, 255);

  Serial.print(sensorValue);
  Serial.print(',');
  Serial.print(pwmV);
  Serial.print(',');
  Serial.print(tgtSpeed);
  Serial.print(',');
  Serial.println(rpm);

  if (sensorValue > center) {
    //    digitalWrite(fwd, HIGH);
    digitalWrite(bac, HIGH);
    analogWrite(pwm, pwmV);
  }
  else if (sensorValue < center) {
    digitalWrite(fwd, HIGH);
    //    digitalWrite(bac, HIGH);
    analogWrite(pwm, pwmV);
  }
  else {
    digitalWrite(fwd, LOW);
    digitalWrite(bac, LOW);
    digitalWrite(pwm, LOW);
  }
}
