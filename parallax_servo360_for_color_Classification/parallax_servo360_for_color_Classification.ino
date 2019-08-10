/* 用改裝後的伺服機PARALLAX servo 搭配極限開關，用以分類顏色之圓盤用*/
#include <Servo.h>

Servo myservo;

void setup()
{
  Serial.begin(9600);
  myservo.attach(3, 750, 2250); // 修正脈衝寬度範圍
  pinMode(7, OUTPUT);
}

void loop()
{
  while (!digitalRead(7)) {
    myservo.write(180); // run
  }
  myservo.writeMicroseconds(1488); // stop
  delay(1000);
  while (digitalRead(7)) {
    myservo.write(180); // run
  }
}
