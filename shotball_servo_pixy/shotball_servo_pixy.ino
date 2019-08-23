#include <Servo.h>

Servo sonic_servoR;
Servo sonic_servoL;

const int shot_ball_pin = 34;
const int Lsonic_servo = 12;
const int Rsonic_servo = 13;
const int angle90 = 49;
const int angle180 = 51;
const int angle135 = 53;

void LRservo90() {
  sonic_servoL.write(15);
  sonic_servoR.write(180);
}
void LRservo180() {
  sonic_servoL.write(105);
  sonic_servoR.write(90);
}
void LRservo135() {
  sonic_servoL.write(60);
  sonic_servoR.write(135);
}
void setup() {
  pinMode(shot_ball_pin, OUTPUT);

  sonic_servoR.attach(Rsonic_servo);
  sonic_servoL.attach(Lsonic_servo);

  pinMode(angle90, INPUT);
  pinMode(angle180, INPUT);
  pinMode(angle135, INPUT);
  // put your setup code here, to run once:

}

void loop() {
  if (digitalRead(angle90)) {
    LRservo90();
  } else if (digitalRead(angle180)) {
    LRservo180();
  } else if (digitalRead(angle135)) {
    LRservo135();
  }
  // put your main code here, to run repeatedly:

}
