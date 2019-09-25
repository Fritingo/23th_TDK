#include <Servo.h>
#include <Pixy2.h>
#include "Wire.h"

#define KS103_L 0x74
#define KS103_R 0x75

Pixy2 pixy;

Servo sonic_servoR;
Servo sonic_servoL;
Servo shotball_servo;
Servo classification_over_servo;
Servo save_ball_servo;

//-------pin----------
//const int angle135 = 53;
//const int angle180 = 51;
const int open_game = 49;
const int team_color_pin = 48;
const int Lsonic_servo = 47 ;
const int shot_ball_pin = 46;
const int move_A = 45;
const int move_B = 44;
const int move_slow = 43;
const int change_pid = 42;



//const int classification_servo_pin = 24;

const int classification_over_servo_pin = 22;

const int Rsonic_servo = 13;
const int shotball_servo_pin = 40;//140 170
const int pixy_color_flag_pin = 39;
const int is_shot_pin = 11;
const int stir_ball_pin = 10;
const int save_ball_servo_pin = 9;

//---------global_var---------
bool lai = 0;
int flag = -1;
unsigned long ks103_time;//
int ks103_state = 0;
int distance_R, distance_L;
bool is_shot = false;
bool is_shot_plus = false;
bool shot_motor_is_ok = false;
bool is_team_color = false;
bool is_first_ball = false;
bool is_start = false;
bool plus_ball_over = false;
int team_color = 1;
int have_team_ball = 0;
bool run_step = false;


unsigned long close_team_color = 0;
unsigned long is_team_color_time = 0;
unsigned long shotservo_time = 0;
unsigned long pidtest_time = 0;

bool ten_ball = false;
//----------func-------------
void setting_ks103(byte addr, byte command) {
  Wire.beginTransmission(addr);
  Wire.write(byte(0x02));
  Wire.write(command);   // 发送降噪指令
  Wire.endTransmission();
  delay(1000);
}

void ks103_update() {
  if (ks103_state == 0) {
    Wire.beginTransmission(KS103_L);
    Wire.write(byte(0x02));
    Wire.write(0xb4);     //量程设置为5m 带温度补偿
    Wire.endTransmission();
    //    ks103_time = millis();
    //    ks103_state++;
    //  } else if ((millis() - ks103_time) > 1 and ks103_state == 1) {
    delay(1);
    Wire.beginTransmission(KS103_L);
    Wire.write(byte(0x02));
    Wire.endTransmission();
    Wire.requestFrom(KS103_L, 2);
    if (2 == Wire.available()) {
      distance_L = Wire.read();
      distance_L =  distance_L << 8;
      distance_L |= Wire.read();
      distance_L = distance_L / 10;
      //      ks103_state++;
      ks103_state = 2;
    }
    ks103_time = millis();
  } else if ((millis() - ks103_time) > 100 and ks103_state == 2) {
    Wire.beginTransmission(KS103_R);
    Wire.write(byte(0x02));
    Wire.write(0xb4);     //量程设置为5m 带温度补偿
    Wire.endTransmission();
    //    ks103_time = millis();
    //    ks103_state++;
    //  } else if ((millis() - ks103_time) > 1 and ks103_state == 3) {
    delay(1);
    Wire.beginTransmission(KS103_R);
    Wire.write(byte(0x02));
    Wire.endTransmission();
    Wire.requestFrom(KS103_R, 2);
    if (2 == Wire.available()) {
      distance_R = Wire.read();
      distance_R =  distance_R << 8;
      distance_R |= Wire.read();
      distance_R = distance_R / 10;
      //      ks103_state++;
      ks103_state = 4;
    }
    ks103_time = millis();
  } else if ((millis() - ks103_time) > 100 and ks103_state == 4) {
    ks103_state = 0;
  }
}

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
  Wire.begin();
  setting_ks103(KS103_L, 0x75);
  setting_ks103(KS103_R, 0x75);
  //  Serial.begin(115200);
  //-------output--------
  pinMode(pixy_color_flag_pin, OUTPUT);
  digitalWrite(pixy_color_flag_pin, HIGH);
  pinMode(shot_ball_pin, OUTPUT);
  digitalWrite(shot_ball_pin, LOW);
  pinMode(stir_ball_pin, OUTPUT);
  digitalWrite(stir_ball_pin, LOW);
  //  pinMode(is_no_ball, OUTPUT);
  //  digitalWrite(is_no_ball, HIGH);

  pinMode(move_A, OUTPUT);
  digitalWrite(move_A, HIGH);
  pinMode(move_B, OUTPUT);
  digitalWrite(move_B, HIGH);
  pinMode(change_pid, OUTPUT);
  digitalWrite(change_pid, HIGH);
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
  pinMode(open_game, INPUT_PULLUP);
  //  pinMode(angle180, INPUT_PULLUP);
  //  pinMode(angle135, INPUT_PULLUP);
  pinMode(team_color_pin, INPUT_PULLUP);
  //  pinMode(is_start_pin, INPUT_PULLUP);
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
  ks103_update();
  //-------check_start------
  if (run_step == false) {
    if (digitalRead(open_game) == LOW) {
      LRservo90();//HIGH 180 LOW 90
      run_step = true;
      flag = 0;
      //      digitalWrite(is_start_pin, LOW);
    }
  }
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
    } else if (ten_ball == true) {
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



  //    if (team_color == 1) {
  //    yello_team();
  //  } else {
  //    orange_team();
  //  }




}

void yello_team() {
  if (distance_L < 110 and flag == 0 and lai == 0) {
    digitalWrite(move_A, HIGH);
    digitalWrite(move_B, LOW);
    pidtest_time = millis();
  } else if (flag == 0) {
    if (millis() - pidtest_time < 800) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

    }
  }

  if (flag == 1) {
    if (millis() - pidtest_time < 3000) {
      digitalWrite(move_A, LOW);
      digitalWrite(move_B, LOW);
    } else {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      flag++;


      pidtest_time = millis();
      digitalWrite(pixy_color_flag_pin, LOW);
    }
  }

  if (flag == 2) {
    if (millis() - pidtest_time < 3000) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
    } else {
      flag++;

    }
  }

  if (distance_R > 70  and flag == 3 and lai == 0) {
    digitalWrite(move_A, HIGH);
    digitalWrite(move_B, LOW);
    pidtest_time = millis();
  } else if (flag == 3) {
    if (millis() - pidtest_time < 800) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      LRservo90();
    }
  }

  if (flag == 4) {
    if (millis() - pidtest_time < 1700) {
      digitalWrite(move_A, LOW);
      digitalWrite(move_B, LOW);
    } else {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      flag++;

      LRservo180();

    }
  }

  if (distance_L > 65 and flag == 5 and lai == 0) {
    digitalWrite(move_A, LOW);
    digitalWrite(move_B, HIGH);
    pidtest_time = millis();
  } else if (flag == 5) {
    if (millis() - pidtest_time < 800) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      LRservo90();
    }
  }

  if (flag == 6) {
    if (millis() - pidtest_time < 1700) {
      digitalWrite(move_A, LOW);
      digitalWrite(move_B, LOW);
    } else {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      flag++;

      LRservo180();
    }
  }

  if (distance_R > 70 and flag == 7 and lai == 0) {
    digitalWrite(move_A, HIGH);
    digitalWrite(move_B, LOW);
    pidtest_time = millis();
  } else if (flag == 7) {
    if (millis() - pidtest_time < 800) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      LRservo90();
    }
  }

  if (flag == 8) {
    if (millis() - pidtest_time < 1700) {
      digitalWrite(move_A, LOW);
      digitalWrite(move_B, LOW);
    } else {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      flag++;

      LRservo180();
    }
  }

  if (distance_L > 65 and flag == 9 and lai == 0) {
    digitalWrite(move_A, LOW);
    digitalWrite(move_B, HIGH);
    pidtest_time = millis();
  } else if (flag == 9) {
    if (millis() - pidtest_time < 800) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      LRservo90();
    }
  }

  if (flag == 10) {
    if (millis() - pidtest_time < 1700) {
      digitalWrite(move_A, LOW);
      digitalWrite(move_B, LOW);
    } else {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      flag++;

      LRservo180();
    }
  }

  if (distance_R > 70 and flag == 11 and lai == 0) {
    digitalWrite(move_A, HIGH);
    digitalWrite(move_B, LOW);
    pidtest_time = millis();
  } else if (flag == 11) {
    if (millis() - pidtest_time < 1000) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      LRservo90();
    }
  }

  if (flag == 12) {
    if (millis() - pidtest_time < 1700) {
      digitalWrite(move_A, LOW);
      digitalWrite(move_B, LOW);
    } else {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      flag++;

      LRservo180();
    }
  }

  if (distance_L > 65 and flag == 13 and lai == 0) {
    digitalWrite(move_A, LOW);
    digitalWrite(move_B, HIGH);
    pidtest_time = millis();
  } else if (flag == 13) {
    if (millis() - pidtest_time < 800) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      LRservo90();
    }
  }

  if (flag == 14) {
    if (millis() - pidtest_time < 1700) {
      digitalWrite(move_A, LOW);
      digitalWrite(move_B, LOW);
    } else {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      flag++;

      LRservo180();
    }
  }

  if (distance_R > 70 and flag == 15 and lai == 0) {
    digitalWrite(move_A, HIGH);
    digitalWrite(move_B, LOW);
    pidtest_time = millis();
  } else if (flag == 15) {
    if (millis() - pidtest_time < 800) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      LRservo90();
    }
  }

  if (flag == 16) {
    if (millis() - pidtest_time < 1700) {
      digitalWrite(move_A, LOW);
      digitalWrite(move_B, LOW);
    } else {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      flag++;

      LRservo180();
    }
  }

  if (distance_L > 65 and flag == 17 and lai == 0) {
    digitalWrite(move_A, LOW);
    digitalWrite(move_B, HIGH);
    pidtest_time = millis();
  } else if (flag == 17) {
    if (millis() - pidtest_time < 800) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      LRservo90();
    }
  }

  if (flag == 18) {
    if (millis() - pidtest_time < 1700) {
      digitalWrite(move_A, LOW);
      digitalWrite(move_B, LOW);
    } else {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      flag++;

      LRservo180();
    }
  }

  if (distance_R > 70 and flag == 19 and lai == 0) {
    digitalWrite(move_A, HIGH);
    digitalWrite(move_B, LOW);
    pidtest_time = millis();
  } else if (flag == 19) {
    if (millis() - pidtest_time < 800) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      LRservo90();
    }
  }

  if (flag == 20) {
    if (millis() - pidtest_time < 1700) {
      digitalWrite(move_A, LOW);
      digitalWrite(move_B, LOW);
    } else {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      flag++;

      LRservo180();
    }
  }

  if (distance_L > 65 and flag == 21 and lai == 0) {
    digitalWrite(move_A, LOW);
    digitalWrite(move_B, HIGH);
    pidtest_time = millis();
  } else if (flag == 21) {
    if (millis() - pidtest_time < 800) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      LRservo90();
    }
  }

  if (flag == 22) {
    if (millis() - pidtest_time < 1700) {
      digitalWrite(move_A, LOW);
      digitalWrite(move_B, LOW);

    } else {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      flag++;
      pidtest_time = millis();

      LRservo180();
    }
  }
  //---------------------------
  if (distance_R > 70 and flag == 23 and lai == 0) {
    digitalWrite(move_A, HIGH);
    digitalWrite(move_B, LOW);
    pidtest_time = millis();
  } else if (flag == 23) {
    if (millis() - pidtest_time < 800) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      LRservo90();
    }
  }

  if (flag == 24) {
    if (millis() - pidtest_time < 1700) {
      digitalWrite(move_A, LOW);
      digitalWrite(move_B, LOW);
    } else {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      flag++;

      LRservo180();
    }
  }

  if (distance_L > 65 and flag == 25 and lai == 0) {
    digitalWrite(move_A, LOW);
    digitalWrite(move_B, HIGH);
    pidtest_time = millis();
  } else if (flag == 25) {
    if (millis() - pidtest_time < 800) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      LRservo90();
    }
  }

  if (flag == 26) {
    if (millis() - pidtest_time < 1700) {
      digitalWrite(move_A, LOW);
      digitalWrite(move_B, LOW);
    } else {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      flag++;
      pidtest_time = millis();

      LRservo180();
    }
  }
  if (distance_R > 70  and flag == 27 and lai == 0) {
    digitalWrite(move_A, HIGH);
    digitalWrite(move_B, LOW);
    pidtest_time = millis();
  } else if (flag == 27) {
    if (millis() - pidtest_time < 800) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      LRservo90();
    }
  }

  if (flag == 28) {
    if (millis() - pidtest_time < 800) {
      digitalWrite(move_A, LOW);
      digitalWrite(move_B, LOW);
    } else {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      flag++;

      LRservo180();

    }
  }

  if (distance_L > 65 and flag == 29 and lai == 0) {
    digitalWrite(move_A, LOW);
    digitalWrite(move_B, HIGH);
    pidtest_time = millis();
  } else if (flag == 29) {
    if (millis() - pidtest_time < 800) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      LRservo90();
    }
  }

  if (flag == 30) {
    if (millis() - pidtest_time < 800) {
      digitalWrite(move_A, LOW);
      digitalWrite(move_B, LOW);
    } else {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      flag++;

      LRservo180();
    }
  }



  //--------find_plus----------
  if (flag == 31) {
    if (millis() - pidtest_time < 1000) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
    }
    else {
      LRservo180();
      flag++;
    }
  }
  //===============flag change==========
  if (distance_R > 260 and flag == 32 and lai == 0)
  {
    digitalWrite(move_A, HIGH);
    digitalWrite(move_B, LOW);
    pidtest_time = millis();
  } else if (distance_R > 130 and distance_R <= 260 and flag == 32 and lai == 0)
  {
    digitalWrite(move_A, HIGH);       digitalWrite(move_B, LOW);       digitalWrite(move_slow, LOW);
    pidtest_time = millis();
  } else if (flag == 32) {
    if (millis() - pidtest_time < 1000) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
      LRservo90();
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      LRservo90();
      digitalWrite(change_pid, LOW);
    }
  }

  if (distance_L > 180 and flag == 33 and lai == 0) {
    digitalWrite(move_A, LOW);       digitalWrite(move_B, LOW);       digitalWrite(move_slow, LOW);
    pidtest_time = millis();
  } else if (flag == 33) {
    if (millis() - pidtest_time < 1000) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      LRservo90();

      ten_ball = true;//射球

    }
  }

  if (relative_yaw < 42 and flag == 34 and lai == 0) {
    LeftAround1();
    pidtest_time = millis();
  } else if (flag == 34) {// and
    if (millis() - pidtest_time < 14000 and plus_ball_over = false) {// or digitalRead(plus_ball_over_pin) == LOW
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;
      //
      //  LRservo90();
    }
  }

  if (relative_yaw > 0.5 and flag == 35 and lai == 0) {
    RightAround1();
    pidtest_time = millis();
  } else if (flag == 35) {
    if (millis() - pidtest_time < 1000) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      LRservo180();
    }
  }

  if (flag == 36) {
    if (millis() - pidtest_time < 1000) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
    }
    else {
      flag++;
    }
  }

  if (distance_R < 130 and flag == 37 and lai == 0)
  {
    digitalWrite(move_A, LOW);
    digitalWrite(move_B, HIGH);
    pidtest_time = millis();
  } else if (distance_R < 295 and distance_R >= 130 and flag == 37 and lai == 0)
  {
    digitalWrite(move_A, LOW);
    digitalWrite(move_B, HIGH);
    digitalWrite(move_slow, LOW);
    pidtest_time = millis();
  } else if (flag == 37) {
    if (millis() - pidtest_time < 1000) {
      digitalWrite(move_A, HIGH); digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;
      digitalWrite(change_pid, LOW);
      LRservo90();
    }
  }

  if (distance_R > 80 and flag == 38 and lai == 0) {
    digitalWrite(move_A, LOW);       digitalWrite(move_B, LOW);       digitalWrite(move_slow, LOW);
    pidtest_time = millis();
  } else if (flag == 38) {
    if (millis() - pidtest_time < 1000) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      LRservo90();
      digitalWrite(is_shot_pin, LOW);//射球
    }
  }

}

void orange_team() {
  if (distance_R < 150 and flag == 0 and lai == 0) {
    digitalWrite(move_A, LOW);
    digitalWrite(move_B, HIGH);
    pidtest_time = millis();
  } else if (flag == 0) {
    if (millis() - pidtest_time < 1000) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;
      LRservo90();
    }
  }

  if (flag == 1) {
    if (millis() - pidtest_time < 3000) {
      digitalWrite(move_A, LOW);
      digitalWrite(move_B, LOW);
    } else {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      flag++;

      LRservo180();
      pidtest_time = millis();
      digitalWrite(pixy_color_flag_pin, LOW);
    }
  }

  if (flag == 2) {
    if (millis() - pidtest_time < 3000) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
    } else {
      flag++;

    }
  }

  if (distance_L > 65 and flag == 3 and lai == 0) {
    digitalWrite(move_A, LOW); digitalWrite(move_B, HIGH);
    pidtest_time = millis();
  } else if (flag == 3) {
    if (millis() - pidtest_time < 800) {
      digitalWrite(move_A, HIGH); digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      LRservo90();
    }
  }

  if (flag == 4) {
    if (millis() - pidtest_time < 1700) {
      digitalWrite(move_A, LOW);
      digitalWrite(move_B, LOW);
    } else {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      flag++;

      LRservo180();

    }
  }

  if (distance_R > 70 and flag == 5 and lai == 0) {
    digitalWrite(move_A, HIGH);
    digitalWrite(move_B, LOW);
    pidtest_time = millis();
  } else if (flag == 5) {
    if (millis() - pidtest_time < 800) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      LRservo90();
    }
  }

  if (flag == 6) {
    if (millis() - pidtest_time < 1700) {
      digitalWrite(move_A, LOW);
      digitalWrite(move_B, LOW);
    } else {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      flag++;

      LRservo180();
    }
  }

  if (distance_L > 65 and flag == 7 and lai == 0) {
    digitalWrite(move_A, LOW);
    digitalWrite(move_B, HIGH);
    pidtest_time = millis();
  } else if (flag == 7) {
    if (millis() - pidtest_time < 800) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      LRservo90();
    }
  }

  if (flag == 8) {
    if (millis() - pidtest_time < 1700) {
      digitalWrite(move_A, LOW);
      digitalWrite(move_B, LOW);
    } else {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      flag++;

      LRservo180();
    }
  }

  if (distance_R > 70 and flag == 9 and lai == 0) {
    digitalWrite(move_A, HIGH);
    digitalWrite(move_B, LOW);
    pidtest_time = millis();
  } else if (flag == 9) {
    if (millis() - pidtest_time < 800) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      LRservo90();
    }
  }

  if (flag == 10) {
    if (millis() - pidtest_time < 1700) {
      digitalWrite(move_A, LOW);
      digitalWrite(move_B, LOW);
    } else {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      flag++;

      LRservo180();
    }
  }

  if (distance_L > 65 and flag == 11 and lai == 0) {
    digitalWrite(move_A, LOW);
    digitalWrite(move_B, HIGH);
    pidtest_time = millis();
  } else if (flag == 11) {
    if (millis() - pidtest_time < 800) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      LRservo90();
    }
  }

  if (flag == 12) {
    if (millis() - pidtest_time < 1700) {
      digitalWrite(move_A, LOW);
      digitalWrite(move_B, LOW);
    } else {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      flag++;

      LRservo180();
    }
  }

  if (distance_R > 70 and flag == 13 and lai == 0) {
    digitalWrite(move_A, HIGH);
    digitalWrite(move_B, LOW);
    pidtest_time = millis();
  } else if (flag == 13) {
    if (millis() - pidtest_time < 800) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      LRservo90();
    }
  }

  if (flag == 14) {
    if (millis() - pidtest_time < 1700) {
      digitalWrite(move_A, LOW);
      digitalWrite(move_B, LOW);
    } else {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      flag++;

      LRservo180();
    }
  }

  if (distance_L > 65 and flag == 15 and lai == 0) {
    digitalWrite(move_A, LOW);
    digitalWrite(move_B, HIGH);
    pidtest_time = millis();
  } else if (flag == 15) {
    if (millis() - pidtest_time < 800) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      LRservo90();
    }
  }

  if (flag == 16) {
    if (millis() - pidtest_time < 1700) {
      digitalWrite(move_A, LOW);
      digitalWrite(move_B, LOW);
    } else {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      flag++;

      LRservo180();
    }
  }

  if (distance_R > 70 and flag == 17 and lai == 0) {
    digitalWrite(move_A, HIGH);
    digitalWrite(move_B, LOW);
    pidtest_time = millis();
  } else if (flag == 17) {
    if (millis() - pidtest_time < 800) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      LRservo90();
    }
  }

  if (flag == 18) {
    if (millis() - pidtest_time < 1700) {
      digitalWrite(move_A, LOW);
      digitalWrite(move_B, LOW);
    } else {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      flag++;

      LRservo180();
    }
  }

  if (distance_L > 65 and flag == 19 and lai == 0) {
    digitalWrite(move_A, LOW);
    digitalWrite(move_B, HIGH);
    pidtest_time = millis();
  } else if (flag == 19) {
    if (millis() - pidtest_time < 800) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      LRservo90();
    }
  }

  if (flag == 20) {
    if (millis() - pidtest_time < 1700) {
      digitalWrite(move_A, LOW);
      digitalWrite(move_B, LOW);
    } else {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      flag++;

      LRservo180();
    }
  }

  if (distance_R > 70 and flag == 21 and lai == 0) {
    digitalWrite(move_A, HIGH);
    digitalWrite(move_B, LOW);
    pidtest_time = millis();
  } else if (flag == 21) {
    if (millis() - pidtest_time < 800) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      LRservo90();
    }
  }

  if (flag == 22) {
    if (millis() - pidtest_time < 1700) {
      digitalWrite(move_A, LOW);
      digitalWrite(move_B, LOW);

    } else {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      flag++;
      pidtest_time = millis();

      LRservo180();
    }
  }
  //---------------------------
  if (distance_L > 65 and flag == 23 and lai == 0) {
    digitalWrite(move_A, LOW);
    digitalWrite(move_B, HIGH);
    pidtest_time = millis();
  } else if (flag == 23) {
    if (millis() - pidtest_time < 800) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      LRservo90();
    }
  }

  if (flag == 24) {
    if (millis() - pidtest_time < 1700) {
      digitalWrite(move_A, LOW);
      digitalWrite(move_B, LOW);
    } else {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      flag++;

      LRservo180();
    }
  }

  if (distance_R > 70 and flag == 25 and lai == 0) {
    digitalWrite(move_A, HIGH);
    digitalWrite(move_B, LOW);
    pidtest_time = millis();
  } else if (flag == 25) {
    if (millis() - pidtest_time < 800) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;

    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      LRservo90();
    }
  }

  if (flag == 26) {
    if (millis() - pidtest_time < 1700) {
      digitalWrite(move_A, LOW);
      digitalWrite(move_B, LOW);
    } else {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      flag++;
      pidtest_time = millis();

      LRservo180();
    }
  }

  //--------find_plus----------
  if (flag == 27) {
    if (millis() - pidtest_time < 1000) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
    }
    else {
      LRservo180();
      flag++;
    }
  }
  //===============flag change==========
  if (distance_L > 250 and flag == 28 and lai == 0)
  {
    digitalWrite(move_A, LOW);
    digitalWrite(move_B, HIGH);
    pidtest_time = millis();
  } else if (distance_L > 130 and distance_L <= 250 and flag == 28 and lai == 0) {
    digitalWrite(move_A, LOW);
    digitalWrite(move_B, HIGH);
    digitalWrite(move_slow, LOW);
    pidtest_time = millis();
  } else if (flag == 28) {
    if (millis() - pidtest_time < 1000) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
      LRservo90();
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;
      digitalWrite(change_pid, LOW);
      LRservo90();
    }
  }

  if (distance_R > 160 and flag == 29 and lai == 0) {
    digitalWrite(move_A, LOW);       digitalWrite(move_B, LOW);       digitalWrite(move_slow, LOW);
    pidtest_time = millis();
  } else if (flag == 29) {
    if (millis() - pidtest_time < 1000) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      LRservo90();
      //      if (digitalRead(is_no_ball) == LOW) {
      //
      //        LRservo180();
      //        flag = 32;
      //      } else {
      digitalWrite(is_shot_plus_pin, LOW);//射球
      //      }
    }
  }

  if (relative_yaw > -46 and flag == 30 and lai == 0) {
    RightAround1();
    pidtest_time = millis();
  } else if (flag == 30) {// and
    if (millis() - pidtest_time < 10000 and plus_ball_over = false) {// or digitalRead(plus_ball_over_pin) == LOW
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;
      //
      //  LRservo90();
    }
  }

  if (relative_yaw < -1 and flag == 31 and lai == 0) {
    LeftAround1();
    pidtest_time = millis();
  } else if (flag == 31) {
    if (millis() - pidtest_time < 1000) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      LRservo180();
    }
  }

  if (flag == 32) {
    if (millis() - pidtest_time < 1000) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
    }
    else {
      flag++;
    }
  }

  if (distance_L < 130 and flag == 33 and lai == 0)
  {
    digitalWrite(move_A, HIGH);
    digitalWrite(move_B, LOW);
    pidtest_time = millis();
  } else if (distance_L < 240 and distance_L >= 130 and flag == 33 and lai == 0) {
    digitalWrite(move_A, HIGH);
    digitalWrite(move_B, LOW);
    digitalWrite(move_slow, LOW);
    pidtest_time = millis();
  } else if (flag == 33) {
    if (millis() - pidtest_time < 1000) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;
      digitalWrite(change_pid, LOW);
      LRservo90();

    }
  }

  if (distance_R > 70 and flag == 34 and lai == 0) {
    digitalWrite(move_A, LOW);
    digitalWrite(move_B, LOW);
    digitalWrite(move_slow, LOW);
    pidtest_time = millis();
  } else if (flag == 34) {
    if (millis() - pidtest_time < 1000) {
      digitalWrite(move_A, HIGH);
      digitalWrite(move_B, HIGH);
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      LRservo90();
      digitalWrite(is_shot_pin, LOW);//射球
    }
  }
}
