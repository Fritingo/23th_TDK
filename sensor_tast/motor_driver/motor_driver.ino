/**
 * @file motor_driver.ino
 * @brief Basic motor driver test sketch.
 *
 * This sketch initializes the motor driver pins and tests individual motors.
 * It contains helper functions to drive each of the 4 motors forward/backward.
 * IR code is commented out but structure remains for remote control integration.
 */

//#include <IRremote.h>

int RECV_PIN = 12; // 使用數位腳位2接收紅外線訊號
//IRrecv irrecv(RECV_PIN); // 初始化紅外線訊號輸入
//decode_results results; // 儲存訊號的結構

const int in1 = 52;
const int in2 = 50;
const int in3 = 48;
const int in4 = 43;
const int in5 = 42;
const int in6 = 41;
const int in7 = 40;
const int in8 = 39;

const int en1 = 46;
const int en2 = 45;
const int en3 = 44;
const int en4 = 10;

const int collect_ball_pin = 35;
const int pullup_ball_pin = 36;
const int shot_ball_pin = 34;

/**
 * @brief Setup function.
 *
 * Initializes output pins for motors and ball mechanism control.
 * Starts Serial communication.
 */
void setup() {
  pinMode(collect_ball_pin, OUTPUT);
  pinMode(pullup_ball_pin, OUTPUT);
  pinMode(shot_ball_pin, OUTPUT);
  
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
  //  irrecv.blink13(true); // 設為true的話，當收到訊號時，腳位13的LED便會閃爍
  //  irrecv.enableIRIn(); // 啟動接收




}

/**
 * @brief Main loop.
 *
 * Tests specific motors by calling their forward functions.
 * Currently configured to run Motor 4 forward at low speed.
 */
void loop() {
  
//  Motor_init();
  Motor4_Forward(50);
//  delay(2000);
//  Motor_init();
//  Motor2_Forward(50);
//  delay(2000);
//  Motor_init();
//  Motor3_Forward(50);
//  delay(2000);
//  Motor_init();
//  Motor4_Forward(50);
//  delay(2000);

  //  if (irrecv.decode(&results)) { // 接收紅外線訊號並解碼
  //    Serial.print("results value is "); // 輸出解碼後的資料//0:16738455/1:16724175/2:16718055/3:16743045/4:16716015/5:16726215/6:16734885/7:16728765/8:16730805/9:16732845
  //    Serial.println(results.value);//0:FF6897/1:FF6897/2:FF18E7/3:FF7A85/4:FF10EF/5:FF38C7/6:FF5AA5/7:FF42BD/8:FF4AB5/9:FF52AD
  //
  //    switch (results.value)
  //    {
  //      case 16738455://0
  //        Motor_init();
  //        Serial.println("000");
  //        break;
  //      case 16724175://1
  //
  //        Serial.println("111");
  //        Serial.println(millis());
  //        Motor1_Forward(220);
  //        delay(5000);
  //        Motor_init();
  //        Serial.println(millis());
  //        break;
  //      case 16718055://2
  //        Motor2_Forward(220);
  //        delay(5000);
  //        Motor_init();
  //        break;
  //      case 16743045://3
  //        Motor3_Forward(220);
  //        delay(5000);
  //        Motor_init();
  //        break;
  //      case 16716015://4
  //        Motor4_Forward(220);
  //        delay(5000);
  //        Motor_init();
  //        break;
  //      case 16726215://5
  //        Motor1_Forward(200);
  //        delay(2000);
  //        Motor_init();
  //        break;
  //      case 16734885://6
  //        Motor1_Forward(200);
  //        delay(2000);
  //        Motor_init();
  //        break;
  //      case 16728765://7
  //        Motor1_Forward(200);
  //        delay(2000);
  //        Motor_init();
  //        break;
  //      case 16730805://8
  //        Motor1_Forward(200);
  //        delay(2000);
  //        Motor_init();
  //        break;
  //      case 16732845://9
  //        Motor1_Forward(200);
  //        delay(2000);
  //        Motor_init();
  //        break;
  //      default:
  //        Motor_init();
  //        break;
  //    }
  //    irrecv.resume(); // 準備接收下一個訊號
//}




}

/**
 * @brief Stops all motors.
 */
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

/**
 * @brief Drives Motor 1 forward.
 *
 * @param Speed PWM speed (0-255).
 */
void Motor1_Forward(int Speed)
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(en1, Speed);
}

/**
 * @brief Drives Motor 1 backward.
 *
 * @param Speed PWM speed (0-255).
 */
void Motor1_Backward(int Speed)
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(en1, Speed);
}

/**
 * @brief Stops Motor 1.
 */
void Motor1_Brake()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

/**
 * @brief Drives Motor 2 forward.
 *
 * @param Speed PWM speed (0-255).
 */
void Motor2_Forward(int Speed)
{
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(en2, Speed);
}

/**
 * @brief Drives Motor 2 backward.
 *
 * @param Speed PWM speed (0-255).
 */
void Motor2_Backward(int Speed)
{
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(en2, Speed);
}

/**
 * @brief Stops Motor 2.
 */
void Motor2_Brake()
{
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

/**
 * @brief Drives Motor 3 forward.
 *
 * @param Speed PWM speed (0-255).
 */
void Motor3_Forward(int Speed)
{
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  analogWrite(en3, Speed);
}

/**
 * @brief Drives Motor 4 forward.
 *
 * @param Speed PWM speed (0-255).
 */
void Motor4_Forward(int Speed)
{
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
  analogWrite(en4, Speed);
}
