#include <Servo.h>
#include <Pixy2.h>

Pixy2 pixy;

Servo sonic_servoR;
Servo sonic_servoL;
Servo shotball_servo;
Servo classification_over_servo;
Servo save_ball_servo;

//-------pin----------
//const int angle135 = 53;
//const int angle180 = 51;
const int angle90 = 49;
const int team_color_pin = 48;
const int Lsonic_servo = 47 ;
const int shot_ball_pin = 46;
const int shot_ball_plus_pin = 45;
//const int is_start_pin = 44;
//const int is_no_ball = 43;
const int plus_ball_over_pin = 42;

//const int classification_servo_pin = 24;

const int classification_over_servo_pin = 22;

const int Rsonic_servo = 13;
const int shotball_servo_pin = 40;//140 170
const int is_shot_pin = 11;
const int stir_ball_pin = 10;
const int save_ball_servo_pin = 9;

//---------global_var---------
bool is_shot = false;
bool is_shot_plus = false;
bool shot_motor_is_ok = false;
bool is_team_color = false;
bool is_first_ball = false;
bool is_start = false;
bool plus_ball_over = false;
int team_color = 1;
int have_team_ball = 0;


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
//  Serial.begin(115200);
  //-------output--------
  pinMode(shot_ball_pin, OUTPUT);
  digitalWrite(shot_ball_pin, LOW);
  pinMode(stir_ball_pin, OUTPUT);
  digitalWrite(stir_ball_pin, LOW);
//  pinMode(is_no_ball, OUTPUT);
//  digitalWrite(is_no_ball, HIGH);
  pinMode(plus_ball_over_pin, OUTPUT);
  digitalWrite(plus_ball_over_pin, HIGH);
  //-------servo---------
  shotball_servo.attach(shotball_servo_pin);
  classification_over_servo.attach(classification_over_servo_pin);
  sonic_servoR.attach(Rsonic_servo);
  sonic_servoL.attach(Lsonic_servo);
  //  classification_servo.attach(classification_servo_pin);
  //  classification_servo.write(92);
  classification_over_servo.write(10);//放40 入球箱10
  shotball_servo.write(170);//壓球170放球140
  save_ball_servo.attach(save_ball_servo_pin);
  save_ball_servo.write(100);
  //------input---------
  pinMode(angle90, INPUT_PULLUP);
//  pinMode(angle180, INPUT_PULLUP);
  //  pinMode(angle135, INPUT_PULLUP);
  pinMode(team_color_pin, INPUT_PULLUP);
  //  pinMode(is_start_pin, INPUT_PULLUP);
  pinMode(is_shot_pin, INPUT_PULLUP);
  pinMode(shot_ball_plus_pin, INPUT_PULLUP);
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
  //  if (is_start == false) {
  //    if (digitalRead(is_start_pin) == LOW) {
  //      is_start = true;
  //      classification_servo.write(100);
  //    }
  //  }
  //--------classification---------
  if (is_shot == false) {
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
          have_team_ball++;
        } else if (millis() - is_team_color_time > 500) {
          classification_over_servo.write(40);
        }
      }
    }
  }
  //----------no_team_ball--------
//  if (have_team_ball != 0 and is_no_ball == LOW) {
//    digitalWrite(is_no_ball, HIGH);
//  }
  //----------shoot---------------
  //---------10_point-------------
  if (plus_ball_over == false and is_shot != true) {
    if (is_shot_plus == true) {
      if (millis() - shotservo_time < 7000 and shot_motor_is_ok == false) {
        shotball_servo.write(170);
      } else if (shot_motor_is_ok == false) {
        shot_motor_is_ok = true;
      } else {
        if (have_team_ball <= 0) {
          classification_over_servo.write(40);
          plus_ball_over == true;
          digitalWrite(plus_ball_over_pin, LOW);
          shotball_servo.write(170);
        } else if (have_team_ball > 3) {
          if (millis() - shotservo_time < 500) {
            shotball_servo.write(140);
          } else if (millis() - shotservo_time < 1300) {
            shotball_servo.write(170);
          } else {
            shotservo_time = millis();
            have_team_ball--;
          }
        } else {
          if (millis() - shotservo_time < 500) {
            shotball_servo.write(140);
          } else if (millis() - shotservo_time < 1300) {
            shotball_servo.write(170);
          } else {
            shotservo_time = millis();
            have_team_ball--;
          }
        }
      }
    } else if (digitalRead(shot_ball_plus_pin) == LOW) {
//      Serial.println("shoot10!!");
      shotservo_time = millis();
      is_shot_plus = true;
      digitalWrite(shot_ball_pin, HIGH);
      digitalWrite(stir_ball_pin, HIGH);
      have_team_ball = have_team_ball + 3;
    }
  }
  //---------------3_point-----------------------
  if (is_shot == true) {
    if (millis() - shotservo_time < 550) {
      save_ball_servo.write(80);
    } else {
      save_ball_servo.write(100);
    }
    if (millis() - shotservo_time < 7000 and shot_motor_is_ok == false) {
      shotball_servo.write(170);
    } else if (shot_motor_is_ok == false) {
      shot_motor_is_ok = true;
    } else {
      if (millis() - shotservo_time < 500) {
        shotball_servo.write(140);
      } else if (millis() - shotservo_time < 1300) {
        shotball_servo.write(170);
      } else {
        shotservo_time = millis();
      }
    }
  } else if (digitalRead(is_shot_pin) == LOW) {
//    Serial.println("shoot!!");
    shotservo_time = millis();
    is_shot = true;
    digitalWrite(stir_ball_pin, HIGH);
    digitalWrite(shot_ball_pin, HIGH);
    classification_over_servo.write(40);
  }
  
  //-----------ks103_servo------------
  if (digitalRead(angle90) == 0) {
    LRservo90();
  } else {
    LRservo180();
  }
  //  if (digitalRead(angle90) == 0) {
  //    LRservo90();
  //  } else if (digitalRead(angle180) == 0) {
  //    LRservo180();
  //  } else if (digitalRead(angle135) == 0) {
  //    LRservo135();



}
