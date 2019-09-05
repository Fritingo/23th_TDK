#include <Servo.h>
#include <Pixy2.h>

//==========pin==========
const int red_servo_pin = 10;
const int green_servo_pin = 11;
const int blue_servo_pin = 12;
const int black_servo_pin = 13;

Pixy2 pixy;

Servo red_servo;  // create servo object to control a servo
Servo green_servo;
Servo blue_servo;
Servo black_servo;

int pos = 0;    // variable to store the servo position


unsigned long start;

int colorxy[3][3] = {{0, 0, 0}, {0, 0, 0}};
int color[3] = {0, 0, 0};
int rg = 0;
int gb = 0;
int rb = 0;
bool get2color = false;

int j;
void reset_color() {
  for (j = 0; j < 3; j++) {
    colorxy[j][0] = 0;
    colorxy[j][1] = 0;
  }
}
void servo_setup() {
  red_servo.write(100);
  green_servo.write(0);
  blue_servo.write(100);
  black_servo.write(100);
}
void setup() {
  Serial.begin(115200);
  red_servo.attach(red_servo_pin);  // 100平 0舉
  green_servo.attach(green_servo_pin);// 100舉 0平
  blue_servo.attach(blue_servo_pin);
  black_servo.attach(black_servo_pin);
  servo_setup();
  pixy.init();
  start = millis();

}
void loop() {
  pixy.ccc.getBlocks();
  if (millis() - start < 2000) {
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      if (pixy.ccc.blocks[i].m_signature == 1) {
        colorxy[0][0] = pixy.ccc.blocks[i].m_x;
        colorxy[0][1] = pixy.ccc.blocks[i].m_y;
      }
      if (pixy.ccc.blocks[i].m_signature == 2) {
        colorxy[1][0] = pixy.ccc.blocks[i].m_x;
        colorxy[1][1] = pixy.ccc.blocks[i].m_y;
      }
      if (pixy.ccc.blocks[i].m_signature == 3) {
        colorxy[2][0] = pixy.ccc.blocks[i].m_x;
        colorxy[2][1] = pixy.ccc.blocks[i].m_y;
      }
      if (abs(colorxy[0][0] - colorxy[1][0]) < 10 and abs(colorxy[0][1] - colorxy[1][1]) > 15) {
        rg++;
        reset_color();
        get2color = true;
      } else if (abs(colorxy[1][0] - colorxy[2][0]) < 10 and abs(colorxy[1][1] - colorxy[2][1]) > 15) {
        gb++;
        reset_color();
        get2color = true;
      } else if (abs(colorxy[2][0] - colorxy[0][0]) < 10 and abs(colorxy[2][1] - colorxy[0][1]) > 15) {
        rb++;
        reset_color();
        get2color = true;
      }
      if (get2color == false) {
        if (pixy.ccc.blocks[i].m_signature == 1) {
          color[0] = 1;
        }
        if (pixy.ccc.blocks[i].m_signature == 2) {
          color[1] = 1;
        }
        if (pixy.ccc.blocks[i].m_signature == 3) {
          color[2] = 1;
        }
      }
    }

    //  if(millis() - start < 10000){
    //  if(pixy.ccc.numBlocks){
    //    for (int i=0; i<pixy.ccc.numBlocks; i++)
    //    {
    //      if(pixy.ccc.blocks[i].m_signature == 1){
    //        Serial.println("red");
    //        red_servo.write(10);
    //        color[0] = 1;
    //      }
    //      if(pixy.ccc.blocks[i].m_signature == 2){
    //        Serial.println("green");
    //        green_servo.write(100);
    //        color[1] = 1;
    //      }
    //      if(pixy.ccc.blocks[i].m_signature == 3){
    //        Serial.println("blue");
    //        blue_servo.write(10);
    //        color[2] = 1;
    //      }
    //    }
    //  }
    //  }else{
    //    if(color[0]+color[1]+color[1]<2){
    //       black_servo.write(10);
    //    }
    //  }


  } else {

    if (get2color == true) {
      if (rg == max(max(rg, gb), rb)) {
        Serial.println("rg");
      } else if (gb == max(max(rg, gb), rb)) {
        Serial.println("gb");
      } else if (rb == max(max(rg, gb), rb)) {
        Serial.println("rb");
      }
    } else {
      if (color[0] == 1) {
        Serial.println("red");
      } else if (color[1] == 1) {
        Serial.println("green");
      } else {
        Serial.println("blue");
      }
      Serial.println("黑");
    }
  }
}
