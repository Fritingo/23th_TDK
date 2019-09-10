#include <Servo.h>
#include <Pixy2.h>

Pixy2 pixy;

Servo sonic_servoR;
Servo sonic_servoL;
Servo classification_servo;
Servo shotball_servo;
Servo classification_over_servo;

//-------pin----------
const int angle135 = 53;
const int angle180 = 51;
const int angle90 = 49;
const int team_color_pin = 48;
const int Lsonic_servo = 47 ;
const int shot_ball_pin = 46;
const int is_start_pin = 45;

const int classification_servo_pin = 24;

const int classification_over_servo_pin = 22;

const int Rsonic_servo = 13;
const int shotball_servo_pin = 12;
const int is_shot_pin = 11;

//---------global_var---------
bool is_shot = false;
bool shot_motor_is_ok = false;
bool is_team_color = false;
int team_color = 1;
bool is_first_ball = false;
bool is_start = false;

unsigned long close_team_color = 0;
unsigned long is_team_color_time = 0;
unsigned long shotservo_time = 0;

//----------func-------------
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
//-----------setup------------
void setup() {
  Serial.begin(115200);
  //-------output--------
  pinMode(shot_ball_pin, OUTPUT);
  digitalWrite(shot_ball_pin, LOW);
  //-------servo---------
  shotball_servo.attach(shotball_servo_pin);
  classification_over_servo.attach(classification_over_servo_pin);
  sonic_servoR.attach(Rsonic_servo);
  sonic_servoL.attach(Lsonic_servo);
  classification_servo.attach(classification_servo_pin);
  classification_servo.write(90);
  classification_over_servo.write(10);//放40 入球箱10
  shotball_servo.write(60);//壓球10擋球60放球90
  //------input---------
  pinMode(angle90, INPUT_PULLUP);
  pinMode(angle180, INPUT_PULLUP);
  pinMode(angle135, INPUT_PULLUP);
  pinMode(team_color_pin, INPUT_PULLUP);
  pinMode(is_start_pin, INPUT_PULLUP);
  pinMode(is_shot_pin, INPUT_PULLUP);
  //------pixy----------
  pixy.init();
  //-----check_bt-------
  delay(3000);
  if (digitalRead(team_color_pin) == LOW) {
    team_color = 1;//黃
  } else {
    team_color = 2;//橘
  }
}

void loop() {
  //-------check_start------
  if (is_start == false) {
    if (is_start_pin == LOW) {
      is_start = true;
      classification_servo.write(100);
    }
  }
  //--------classification---------
  pixy.ccc.getBlocks();
  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    //      Serial.println(pixy.ccc.blocks[i].m_signature);
    if (pixy.ccc.blocks[i].m_signature == team_color and pixy.ccc.blocks[i].m_x < 125) {
      classification_over_servo.write(10);
      is_team_color = true;
      close_team_color = millis();

    }
  }
  if (is_team_color == true) {
    if (is_first_ball == false) {
      is_team_color_time = millis();
      close_team_color = millis();
      is_first_ball = true;
    } else {
      if (millis() - close_team_color > 600) {
        is_team_color = false;
        is_first_ball = false;
        classification_over_servo.write(10);
      } else if (millis() - is_team_color_time > 500) {
        classification_over_servo.write(40);
      }
    }
  }
//----------shoot---------------
  if (is_shot == true) {
    if (millis() - shotservo_time < 5000 and shot_motor_is_ok == false) {
      shotball_servo.write(60);
    } else if (shot_motor_is_ok == false) {
      shot_motor_is_ok = true;
    } else {
      if (millis() - shotservo_time < 500) {
        shotball_servo.write(60);
      } else if (millis() - shotservo_time < 980) {
        shotball_servo.write(105);
      } else if (millis() - shotservo_time < 1500) {
        shotball_servo.write(60);
      } else if (millis() - shotservo_time < 2000) {
        shotball_servo.write(10);
      } else {
        shotservo_time = millis();
      }
    }
  } else if (digitalRead(is_shot_pin) == LOW) {
    Serial.println("shoot!!");
    shotservo_time = millis();
    is_shot = true;
    digitalWrite(shot_ball_pin, HIGH);
  }
//-----------ks103_servo------------
  if (digitalRead(angle90) == 0) {
    LRservo90();

  } else if (digitalRead(angle180) == 0) {
    LRservo180();
  } else if (digitalRead(angle135) == 0) {
    LRservo135();
  }
  // put your main code here, to run repeatedly:

}
