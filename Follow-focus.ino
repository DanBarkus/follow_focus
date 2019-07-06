#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <PID_v1.h>

Encoder myEnc(2, 3);

static int fwd = 5;
static int bac = 6;
static int pwm = 9;
static int center = 515;

const int numReadings = 10;

double readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
double total = 0;                  // the running total
double average = 0;                // the average

long oldPosition  = 0;
long oldTime = 0;
int thresh = 1 * 1000;
double tgtSpeed;
double tgtSpeedPID;
double rpm = 0;
double rpmPID = 0;
double pwmV = 0;
// pwm settings
double Kp=1.1, Ki=20, Kd=0.0;
double Kps=0.8, Kis=8, Kds=0.0;
PID myPID(&rpmPID, &pwmV, &tgtSpeedPID, Kp, Ki, Kd, DIRECT);

// the setup routine runs once when you press reset:
void setup() {
  pinMode(fwd, OUTPUT);
  pinMode(bac, OUTPUT);
  pinMode(pwm, OUTPUT);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);
  TCCR1B = TCCR1B & 0b11111000 | 0x02;

  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0.0;
  }
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  int sensorValue = analogRead(A0);
  int currentValue = analogRead(A9);
  tgtSpeed = abs(sensorValue - center);
  if (tgtSpeed < 2) {
    tgtSpeed = 0;
  }
  tgtSpeed = map(tgtSpeed, 0, 512, 0, 24000);
  tgtSpeedPID = map(tgtSpeed, 0, 24000, 0, 255);
  long newTime = micros();
  long deltaT = newTime - oldTime;
  long newPosition = myEnc.read();
  myEnc.write(0);
  oldTime = micros();
  // (change in position / encoder tics per rev) * micros in a second / change in time
  rpm = (abs(newPosition - oldPosition) / 12.0) * (60000.0 * 1000) / (deltaT * 1.0);
  total = total - readings[readIndex];
  readings[readIndex] = rpm;
  total = total + readings[readIndex];
  readIndex = readIndex + 1;

  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  rpm = total / numReadings;
  rpmPID = map(rpm, 0, 24000, 0, 255);
  /*
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
  */
  if (tgtSpeed > 200) {
    if (abs(rpm - tgtSpeed) > 1600) {
      myPID.SetTunings(Kp,Ki,Kd);
    }
    /*
    else {
      myPID.SetTunings(Kps,Kis,Kds);
    }
    */
    myPID.Compute();
  }

  Serial.print(sensorValue/5);
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
