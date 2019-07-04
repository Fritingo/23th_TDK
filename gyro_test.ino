//#include "MeOrion.h"
#include <IRremote.h>

#include <Wire.h>


IRrecv irrecv(11); // 使用數位腳位11接收紅外線訊號初始化紅外線訊號輸入
decode_results results; // 儲存訊號的結構


unsigned long start_time;
unsigned long before;
unsigned long now;
unsigned long initial;
unsigned long current;
const int in1 = 52;
const int in2 = 50;
const int in3 = 48;
const int in4 = 46;
const int in5 = 44;
const int in6 = 42;
const int in7 = 40;
const int in8 = 38;

const int en1 = 2;
const int en2 = 3;
const int en3 = 4;
const int en4 = 5;

float orignal_z;
//MeGyro gyro;


void setup() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(in5, OUTPUT);
  pinMode(in6, OUTPUT);
  pinMode(in7, OUTPUT);
  pinMode(in8, OUTPUT);

  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);
  pinMode(en3, OUTPUT);
  pinMode(en4, OUTPUT);

  Serial.begin(115200);
  irrecv.blink13(true); // 設為true的話，當收到訊號時，腳位13的LED便會閃爍
  irrecv.enableIRIn(); // 啟動接收
  Serial.println("start");
  //  ==========陀螺儀==========
//  gyro.begin();
//  gyro.update();
//  Serial.read();
//  orignal_z = gyro.getAngleZ();//get 初始角度
  //  =========================
  Motor_init();

}

void loop() {

//  gyro.update();
//  Serial.read();
//  Serial.print("orignal_z:");
//  Serial.println(orignal_z);
//  Serial.print(" Z:");
//  Serial.println(gyro.getAngleZ() );
//  Serial.print(" end:");
//  Serial.println(gyro.getAngleZ() - orignal_z );

  if (irrecv.decode(&results)) { // 接收紅外線訊號並解碼
    Serial.print("results value is "); // 輸出解碼後的資料//0:16738455/1:16724175/2:16718055/3:16743045/4:16716015/5:16726215/6:16734885/7:16728765/8:16730805/9:16732845
    Serial.println(results.value);//0:FF6897/1:FF6897/2:FF18E7/3:FF7A85/4:FF10EF/5:FF38C7/6:FF5AA5/7:FF42BD/8:FF4AB5/9:FF52AD

    switch (results.value)
    {
      case 16738455://0
        Motor_init();
        Serial.println("000");
        break;
      case 16724175://1

        Serial.println("111");
        Serial.println(millis());
        m_type_Forward(100, 1000);
        Motor_init();
        Serial.println(millis());
        break;
      case 16718055://2
        m_type_Backward(100, 1000);
        delay(1000);
        Motor_init();
        break;
      case 16743045://3
        m_type_Leftward(100, 1000);
        delay(1000);
        Motor_init();
        //        Serial.println("333");
        //        Motor3_Forward(220);
        //        delay(5000);
        //        Motor_init();
        break;
      case 16716015://4
        m_type_Rightward(100, 1000);
        delay(1000);
        Motor_init();
        //        Motor4_Forward(220);
        //        delay(5000);
        //                Motor_init();
        break;
      case 16726215://5
        m_type_RightAround(50, 1000);
        Motor_init();
        break;
      case 16734885://6
        m_type_LeftAround(50, 1000);
        Motor_init();
        break;
      case 16728765://7
        Motor_start(200);
        //        Motor1_Forward(200);
        //        delay(2000);
        Motor_init();
        break;
      case 16730805://8
        Motor_brakes(200);
        //        Motor1_Forward(200);
        //        delay(2000);
        Motor_init();
        break;
      case 16732845://9
        //        Motor1_Forward(200);
        //        delay(2000);
        Motor_init();
        break;
      default:
        Motor_init();
        break;
    }
    irrecv.resume(); // 準備接收下一個訊號
  }




}
void Motor_init()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  digitalWrite(in5, LOW);
  digitalWrite(in6, LOW);
  digitalWrite(in7, LOW);
  digitalWrite(in8, LOW);
}
void Motor_start(int Speed)
{
  initial = millis();
  for (int i = 1; i <= Speed / 10 ; i++)
  {
    do
    {
      current = millis();
      analogWrite(en1, i * 10);
      analogWrite(en2, i * 10);
      analogWrite(en3, i * 10);
      analogWrite(en4, i * 10);
    } while (current - initial < 100);
    initial = millis();
  }
}
void Motor_brakes(int Speed)
{
  initial = millis();
  for (int i = Speed / 10; i >= 0 ; i--)
  {
    do
    {
      current = millis();
      analogWrite(en1, i * 10);
      analogWrite(en2, i * 10);
      analogWrite(en3, i * 10);
      analogWrite(en4, i * 10);
    } while (current - initial < 100);
    initial = millis();
  }
  Motor_init();
}
void Motor_full_work(int Speed, int Time)
{
  unsigned long motor_full_start = millis();
  unsigned long motor_full_now = millis();
  do {
    analogWrite(en1, Speed);
    analogWrite(en2, Speed);
    analogWrite(en3, Speed);
    analogWrite(en4, Speed);
    motor_full_now = millis();
  } while (motor_full_now - motor_full_start < Time);

}
void m_type_Forward(int Speed, int Time)
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
  Motor_start(Speed);
  Motor_full_work(Speed, Time);
  Motor_brakes(Speed);
}
void m_type_Backward(int Speed, int Time)
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);
  Motor_start(Speed);
  Motor_full_work(Speed, Time);
  Motor_brakes(Speed);
}
void m_type_Rightward(int Speed, int Time)
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
  Motor_start(Speed);
  Motor_full_work(Speed, Time);
  Motor_brakes(Speed);
}
void m_type_Leftward(int Speed, int Time)
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);
  Motor_start(Speed);
  Motor_full_work(Speed, Time);
  Motor_brakes(Speed);
}
void m_type_LeftAround(int Speed, int Time)
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);
  Motor_start(Speed);
  Motor_full_work(Speed, Time);
  Motor_brakes(Speed);
}
void m_type_RightAround(int Speed, int Time)
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
  Motor_start(Speed);
  Motor_full_work(Speed, Time);
  Motor_brakes(Speed);
}
//void m_type_correction_angle()
//{
//  if ((gyro.getAngleZ() - orignal_z) > 5.0)
//  {
//    do {
//      m_type_RightAround(50, 0);
//    } while ((gyro.getAngleZ() - orignal_z) > 5.0);
//  } else if ((gyro.getAngleZ() - orignal_z) < -5.0)
//  {
//    do {
//      m_type_LeftAround(50, 0);
//    } while ((gyro.getAngleZ() - orignal_z) < -5.0);
//  } else {
//    m_type_Forward(50, 0);
//  }
//}
void Motor1_Forward(int Speed)
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(en1, Speed);
}

void Motor1_Backward(int Speed)
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(en1, Speed);
}
void Motor1_Brake()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}
void Motor2_Forward(int Speed)
{
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(en2, Speed);
}

void Motor2_Backward(int Speed)
{
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(en2, Speed);
}
void Motor2_Brake()
{
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
void Motor3_Forward(int Speed)
{
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  analogWrite(en3, Speed);
}
void Motor4_Forward(int Speed)
{
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
  analogWrite(en4, Speed);
}
