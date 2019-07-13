#include <Servo.h>
#include <Pixy2.h>

Pixy2 pixy;

Servo servo1;  // create servo object to control a servo
Servo servo2;
Servo servo3;
Servo servo4;

int pos = 0;    // variable to store the servo position

unsigned long start;

int color[3] = {0,0,0};


void servo_setup() {
  servo1.write(180);
  servo2.write(180);
  servo3.write(180);
  servo4.write(180);
}
void setup() {
  Serial.begin(115200);
  servo1.attach(8);  // attaches the servo on pin 9 to the servo object
  servo2.attach(9);
  servo3.attach(10);
  servo4.attach(11);
  servo_setup();
  pixy.init();
  start = millis();
  
}
void loop() {
  pixy.ccc.getBlocks();

  if(millis() - start < 10000){
  if(pixy.ccc.numBlocks){
    for (int i=0; i<pixy.ccc.numBlocks; i++)
    {
      if(pixy.ccc.blocks[i].m_signature == 1){
        servo1.write(90);
        color[0] = 1;
      }
      if(pixy.ccc.blocks[i].m_signature == 2){
        servo2.write(90);
        color[1] = 1;
      }
      if(pixy.ccc.blocks[i].m_signature == 3){
        servo3.write(90);
        color[2] = 1;
      }  
    }
  }
  }else{
    if(color[0]+color[1]+color[1]<2){
       servo4.write(90);
    }
  }
 
  
}
