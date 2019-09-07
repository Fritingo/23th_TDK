#include <Servo.h>

Servo sonic_servoR;
Servo sonic_servoL;
Servo classification_servo;
Servo shotball_servo;
Servo classification_over_servo;

const int shot_ball_pin = 50;
const int shotball_servo_pin = 12;
const int Lsonic_servo =47 ;
const int Rsonic_servo = 13;
const int classification_servo_pin = 52;
const int angle90 = 49;
const int angle180 = 51;
const int angle135 = 53;
const int classification_over_servo_pin = 22;

unsigned long shotservo_time = 0;

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
  shotball_servo.attach(shotball_servo_pin);

  classification_over_servo.attach(classification_over_servo_pin);
  sonic_servoR.attach(Rsonic_servo);
  sonic_servoL.attach(Lsonic_servo);
  
  classification_servo.attach(classification_servo_pin);
  
  pinMode(angle90, INPUT);
  pinMode(angle180, INPUT);
  pinMode(angle135, INPUT);
  classification_servo.write(130);
  classification_over_servo.write(10);//放40 入球箱10
  shotball_servo.write(100);//壓球10擋球60放球90
  // put your setup code here, to run once:
  shotservo_time = millis();
}

void loop() {
  if(millis()-shotservo_time<500){
    shotball_servo.write(60);
  }else if(millis()-shotservo_time<980){
    shotball_servo.write(105);
  }else if(millis()-shotservo_time<1500){
    shotball_servo.write(60);
  }else if(millis()-shotservo_time<2000){
    shotball_servo.write(10);
  }else{
    shotservo_time = millis();
  }
  if (digitalRead(angle90)) {
    LRservo90();
  } else if (digitalRead(angle180)) {
    LRservo180();
  } else if (digitalRead(angle135)) {
    LRservo135();
  }
  // put your main code here, to run repeatedly:

}
