#include <Servo.h>
#include <Pixy2.h>

//==========pin==========
const int red_servo_pin = 36;
const int green_servo_pin = 34;
const int blue_servo_pin = 32;
const int black_servo_pin = 30;
const int pixy_color_flag_pin = 25;//orignal 25 without_ks103 26
const int collect_ball_pin = 22;
const int pullup_ball_pin = 23;
const int classification_servo_pin = 24;

Pixy2 pixy;

Servo red_servo;  // create servo object to control a servo
Servo green_servo;
Servo blue_servo;
Servo black_servo;
Servo classification_servo;

int pos = 0;    // variable to store the servo position


unsigned long start;

int colorxy[3][3] = {{0, 0, 0}, {0, 0, 0}};
int color[3] = {0, 0, 0};
int rg = 0;
int gb = 0;
int rb = 0;
bool get2color = false;
bool is_identification = false;

int j;
void reset_color() {
  for (j = 0; j < 3; j++) {
    colorxy[j][0] = 0;
    colorxy[j][1] = 0;
  }
}
void servo_setup() {
  red_servo.write(90);// 90平 60舉
  green_servo.write(0); //0平 45舉
  blue_servo.write(90);// 90平 45舉
  black_servo.write(0);//0平 30舉
}

void servo_all_up() {
  red_servo.write(60);// 90平 60舉
  green_servo.write(45); //0平 45舉
  blue_servo.write(45);// 90平 45舉
  black_servo.write(30);//0平 30舉
}

void rg_servo() {
  red_servo.write(60);// 90平 60舉
  green_servo.write(45); //0平 45舉
  blue_servo.write(90);// 90平 45舉
  black_servo.write(0);//0平 30舉
}
void gb_servo() {
  red_servo.write(90);// 90平 60舉
  green_servo.write(45); //0平 45舉
  blue_servo.write(45);// 90平 45舉
  black_servo.write(0);//0平 30舉
}
void rb_servo() {
  red_servo.write(60);// 90平 60舉
  green_servo.write(0); //0平 45舉
  blue_servo.write(45);// 90平 45舉
  black_servo.write(0);//0平 30舉
}
void rblack_servo() {
  red_servo.write(60);// 90平 60舉
  green_servo.write(0); //0平 45舉
  blue_servo.write(90);// 90平 45舉
  black_servo.write(30);//0平 30舉
}
void gblack_servo() {
  red_servo.write(90);// 90平 60舉
  green_servo.write(45); //0平 45舉
  blue_servo.write(90);// 90平 45舉
  black_servo.write(30);//0平 30舉
}
void bblack_servo() {
  red_servo.write(90);// 90平 60舉
  green_servo.write(0); //0平 45舉
  blue_servo.write(45);// 90平 45舉
  black_servo.write(30);//0平 30舉
}
void setup() {
  //  Serial.begin(115200);
  red_servo.attach(red_servo_pin);
  green_servo.attach(green_servo_pin);
  blue_servo.attach(blue_servo_pin);
  black_servo.attach(black_servo_pin);
  servo_all_up();
  pinMode(pixy_color_flag_pin, INPUT_PULLUP);
  pinMode(collect_ball_pin, OUTPUT);
  pinMode(pullup_ball_pin, OUTPUT);
  digitalWrite(collect_ball_pin, LOW);
  digitalWrite(pullup_ball_pin, LOW);

  pixy.init();
  classification_servo.attach(classification_servo_pin);
  classification_servo.write(92);
  delay(5000);
  servo_setup();
}
void loop() {
  if (digitalRead(pixy_color_flag_pin) == 0) {
    red_servo.write(90);// 90平 60舉
    green_servo.write(0); //0平 45舉
    blue_servo.write(90);// 90平 45舉
    black_servo.write(0);//0平 30舉
    digitalWrite(collect_ball_pin, HIGH);
    digitalWrite(pullup_ball_pin, HIGH);
    classification_servo.write(100);
  } else {
    red_servo.write(60);// 90平 60舉
    green_servo.write(45); //0平 45舉
    blue_servo.write(45);// 90平 45舉
    black_servo.write(30);//0平 30舉
  }
  //  if (digitalRead(pixy_color_flag_pin) == 0 and is_identification == false) {
  //    is_identification = true;
  //    start = millis();
  //  }
  //  if (is_identification == true) {
  //    pixy.ccc.getBlocks();
  //    if (millis() - start < 2000) {
  //      for (int i = 0; i < pixy.ccc.numBlocks; i++) {
  //        if (pixy.ccc.blocks[i].m_signature == 1) {
  //          colorxy[0][0] = pixy.ccc.blocks[i].m_x;
  //          colorxy[0][1] = pixy.ccc.blocks[i].m_y;
  //        }
  //        if (pixy.ccc.blocks[i].m_signature == 2) {
  //          colorxy[1][0] = pixy.ccc.blocks[i].m_x;
  //          colorxy[1][1] = pixy.ccc.blocks[i].m_y;
  //        }
  //        if (pixy.ccc.blocks[i].m_signature == 3) {
  //          colorxy[2][0] = pixy.ccc.blocks[i].m_x;
  //          colorxy[2][1] = pixy.ccc.blocks[i].m_y;
  //        }
  //        if (abs(colorxy[0][0] - colorxy[1][0]) < 10 and abs(colorxy[0][1] - colorxy[1][1]) > 15) {
  //          rg++;
  //          reset_color();
  //          get2color = true;
  //        } else if (abs(colorxy[1][0] - colorxy[2][0]) < 10 and abs(colorxy[1][1] - colorxy[2][1]) > 15) {
  //          gb++;
  //          reset_color();
  //          get2color = true;
  //        } else if (abs(colorxy[2][0] - colorxy[0][0]) < 10 and abs(colorxy[2][1] - colorxy[0][1]) > 15) {
  //          rb++;
  //          reset_color();
  //          get2color = true;
  //        }
  //        if (get2color == false) {
  //          if (pixy.ccc.blocks[i].m_signature == 1) {
  //            color[0] = 1;
  //          }
  //          if (pixy.ccc.blocks[i].m_signature == 2) {
  //            color[1] = 1;
  //          }
  //          if (pixy.ccc.blocks[i].m_signature == 3) {
  //            color[2] = 1;
  //          }
  //        }
  //      }
  //    } else {
  //      digitalWrite(collect_ball_pin, HIGH);
  //      digitalWrite(pullup_ball_pin, HIGH);
  //      classification_servo.write(100);
  //      if (get2color == true) {
  //        if (rg == max(max(rg, gb), rb)) {
  //          rg_servo();
  ////          Serial.println("rg");
  //        } else if (gb == max(max(rg, gb), rb)) {
  //          gb_servo();
  ////          Serial.println("gb");
  //        } else if (rb == max(max(rg, gb), rb)) {
  //          rb_servo();
  ////          Serial.println("rb");
  //        }
  //      } else {
  //        if (color[0] == 1) {
  //          rblack_servo();
  ////          Serial.println("red");
  //        } else if (color[1] == 1) {
  //          gblack_servo();
  ////          Serial.println("green");
  //        } else {
  //          bblack_servo();
  ////          Serial.println("blue");
  //        }
  ////        Serial.println("黑");
  //      }
  //      //      digitalWrite(collect_ball_pin,HIGH);
  //      //      digitalWrite(pullup_ball_pin,HIGH);
  //    }
  //  }
}
