/**
 * @file tdk23_ks103_distance_test.ino
 * @brief Robot control firmware focused on distance sensor testing and integration.
 *
 * This file is very similar to `tdk23_openmv15.ino` but seems to be a variation or an earlier version
 * focused on testing KS103 sensors and movement logic.
 * It contains PID control, MPU6050 integration, and team strategies.
 */

#include "Simple_MPU6050.h"
#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

Simple_MPU6050 mpu;
ENABLE_MPU_OVERFLOW_PROTECTION();
//============================
#include "Wire.h"

#define KS103_L 0x74
#define KS103_R 0x75

//----------gobal_var---------
unsigned long ks103_time;//
int ks103_state = 0;
int distance_R, distance_L;
unsigned long step_start;
unsigned long initial;
float original_z = 0;
bool gyro_ready = false;
float relative_yaw;
float relative_yaw1;
float relative_yaw2;
float base_yaw;
float goal_yaw;
int team_color = 1;
bool run_step = false;
bool led_state = false;
int orange_f_time = 900; //900
int yellow_f_time = 900; //900 //1000
int cross_f_time = 1550; //1550 //1600
float relative_yaw_error = 0;
bool check_x = 0;
bool check_y = 0;

//----------pin-----------
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
const int angle90 = 49;
//const int angle180 = 51;
//const int angle135 = 53;
const int Buzzer1 = 38;
const int Buzzer2 = 36;
const int pixy_color_flag_pin = 37;
const int is_shot_pin = 27;
const int team_color_pin = 34;
const int start_bt_pin = 35;
//const int riseball_pin = 33;
//const int sweepball_pin = 32;
//-----------------------------
const int is_shot_plus_pin = 31;
//const int is_start_pin = 30;
//const int is_no_ball = 29;
const int plus_ball_over_pin = 28;
//-----------------------------
#define spamtimer(t) for (static uint32_t SpamTimer; (uint32_t)(millis() - SpamTimer) >= (t); SpamTimer = millis()) // (BLACK BOX) Ya, don't complain that I used "for(;;){}" instead of "if(){}" for my Blink Without Delay Timer macro. It works nicely!!!
#define printfloatx(Name,Variable,Spaces,Precision,EndTxt) print(Name); {char S[(Spaces + Precision + 3)];Serial.print(F(" ")); Serial.print(dtostrf((float)Variable,Spaces,Precision ,S));}Serial.print(EndTxt);//Name,Variable,Spaces,Precision,EndTxt

/**
 * @brief Callback for MPU6050 FIFO data processing.
 *
 * Calculates relative yaw from quaternion data.
 */
void get_yaw(int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp) {
  Quaternion q;
  VectorFloat gravity;
  float ypr[3] = { 0, 0, 0 };
  float xyz[3] = { 0, 0, 0 };
  spamtimer(100) {// non blocking delay before printing again. This skips the following code when delay time (ms) hasn't been met
    mpu.GetQuaternion(&q, quat);
    mpu.GetGravity(&gravity, &q);
    mpu.GetYawPitchRoll(ypr, &q, &gravity);
    mpu.ConvertToDegrees(ypr, xyz);
    relative_yaw = (ypr[0] * 180) / PI;
    relative_yaw = relative_yaw - relative_yaw_error;
  }
}
//----------pid------------
int kp = 5;
int kd = 3;
int kp1 = 5;
int kd1 = 3;
int speed_n = 100;
int speed_n3 = 100;
int speed_ne3 = 90;
int speed_pu3 = 110;
int speed_ne = -110;
int speed_pu = 110;
int speed_n2 = 50;
int speed_ne2 = 40;
int speed_pu2 = 60;
int speed_n1 = 180;
int speed_ne1 = 170;
int speed_pu1 = 190;
int e;
int e1;
int e_pre = 0;
int control;
int control1;
int speed_L;
int speed_R;
int speed_LI;
int speed_RI;
int speed_L1;
int speed_R1;
int speed_LI1;
int speed_RI1;
int flag = -1;
unsigned long pidtest_time;
int lai = 0;
int lai1 = 0;
//unsigned long lai2 = 0;
int lai3 = 2;
int STOP1 = 1;
//---------func-----------

/**
 * @brief PID control for rightward movement (variant 0).
 */
void PIDR() {
  e = relative_yaw - relative_yaw1;
  e1 = abs(e);
  control = kp * e1 + kd * (e1 - e_pre);
  speed_L = speed_n3 + control;
  // speed_R = speed_n - control;
  if (speed_L > speed_pu3) {
    speed_L = speed_pu3;
  }
  //  if (speed_L < speed_ne) {
  //    speed_L = speed_ne;
  //  }
  if (speed_R > speed_pu3) {
    speed_R = speed_pu3;
  }
  //  if (speed_R < speed_ne) {
  //    speed_R = speed_ne;
  //  }
  speed_L = abs(speed_L);
  //  speed_R = abs(speed_R);
  speed_LI = floor(speed_L);
  //  speed_RI = floor(speed_R);
  e_pre = e1;
  if (e > 2) {
    RightAround();
  }
  else if (e < -2) {
    LeftAround();
  }
  else {
    Rightward();
  }
}

/**
 * @brief PID control for rightward movement (variant 1).
 */
void PIDR1() {
  e = relative_yaw - relative_yaw1;
  // e1 = abs(e);
  control = kp * e + kd * (e - e_pre);
  speed_L = speed_n3 + control;
  speed_R = speed_n3 - control;
  if (speed_L > speed_pu3) {
    speed_L = speed_pu3;
  }
  if (speed_L < speed_ne3) {
    speed_L = speed_ne3;
  }
  if (speed_R > speed_pu3) {
    speed_R = speed_pu3;
  }
  if (speed_R < speed_ne3) {
    speed_R = speed_ne3;
  }
  //  speed_L = abs(speed_L);
  //    speed_R = abs(speed_R);
  speed_LI = floor(speed_L);
  speed_RI = floor(speed_R);
  e_pre = e;
  if (e > 0 or e < 0) {
    RightAround();
  }
  else {
    Rightward();
  }
}

/**
 * @brief PID control for rightward movement (variant 2).
 */
void PIDR2() {
  e = relative_yaw - relative_yaw1;
  //  e1 = abs(e);
  control = kp * e + kd * (e - e_pre);
  speed_L = speed_n2 + control;
  speed_R = speed_n2 - control;
  if (speed_L > speed_pu2) {
    speed_L = speed_pu2;
  }
  if (speed_L < speed_ne2) {
    speed_L = speed_ne2;
  }
  if (speed_R > speed_pu2) {
    speed_R = speed_pu2;
  }
  if (speed_R < speed_ne2) {
    speed_R = speed_ne2;
  }
  // speed_L = abs(speed_L);
  //  speed_R = abs(speed_R);
  speed_LI = floor(speed_L);
  speed_RI = floor(speed_R);
  e_pre = e;
  if (e > 0 or e < 0) {
    RightAround();
  }
  else {
    Rightward2();
  }
}

/**
 * @brief PID control for leftward movement (variant 0).
 */
void PIDL() {
  e = relative_yaw - relative_yaw1;
  e1 = abs(e);
  control = kp * e1 + kd * (e1 - e_pre);
  speed_L = speed_n3 + control;
  //  speed_R = speed_n - control;
  if (speed_L > speed_pu3) {
    speed_L = speed_pu3;
  }
  //  if (speed_L < speed_ne) {
  //    speed_L = speed_ne;
  //  }
  if (speed_R > speed_pu3) {
    speed_R = speed_pu3;
  }
  //  if (speed_R < speed_ne) {
  //    speed_R = speed_ne;
  //  }
  speed_L = abs(speed_L);
  //  speed_R = abs(speed_R);
  speed_LI = floor(speed_L);
  //  speed_RI = floor(speed_R);
  e_pre = e1;
  if (e > 2) {
    RightAround();
  }
  else if (e < -2) {
    LeftAround();
  }
  else {
    Leftward();
  }
}

/**
 * @brief PID control for leftward movement (variant 1).
 */
void PIDL1() {
  e = relative_yaw - relative_yaw1;
  //  e1 = abs(e);
  control = kp * e + kd * (e - e_pre);
  speed_L = speed_n3 + control;
  speed_R = speed_n3 - control;
  if (speed_L > speed_pu3) {
    speed_L = speed_pu3;
  }
  if (speed_L < speed_ne3) {
    speed_L = speed_ne3;
  }
  if (speed_R > speed_pu3) {
    speed_R = speed_pu3;
  }
  if (speed_R < speed_ne3) {
    speed_R = speed_ne3;
  }
  //speed_L = abs(speed_L);
  //  speed_R = abs(speed_R);
  speed_LI = floor(speed_L);
  speed_RI = floor(speed_R);
  e_pre = e;
  if (e > 0 or e < 0) {
    LeftAround();
  }
  else {
    Leftward();
  }
}

/**
 * @brief PID control for leftward movement (variant 2).
 */
void PIDL2() {
  e = relative_yaw - relative_yaw1;
  // e1 = abs(e);
  control = kp * e + kd * (e - e_pre);
  speed_L = speed_n2 + control;
  speed_R = speed_n2 - control;
  if (speed_L > speed_pu2) {
    speed_L = speed_pu2;
  }
  if (speed_L < speed_ne2) {
    speed_L = speed_ne2;
  }
  if (speed_R > speed_pu2) {
    speed_R = speed_pu2;
  }
  if (speed_R < speed_ne2) {
    speed_R = speed_ne2;
  }
  // speed_L = abs(speed_L);
  //  speed_R = abs(speed_R);
  speed_LI = floor(speed_L);
  speed_RI = floor(speed_R);
  e_pre = e;
  if (e > 0 or e < 0) {
    LeftAround();
  }
  else {
    Leftward2();
  }
}

/**
 * @brief PID control for forward movement.
 */
void PIDF() {
  e = relative_yaw - relative_yaw1;
  //  e1 = abs(e);
  control1 = kp1 * e + kd1 * (e - e_pre);
  speed_L1 = speed_n1 + control1;
  speed_R1 = speed_n1 - control1;
  if (speed_L1 > speed_pu1) {
    speed_L1 = speed_pu1;
  }
  if (speed_L1 < speed_ne1) {
    speed_L1 = speed_ne1;
  }
  if (speed_R1 > speed_pu1) {
    speed_R1 = speed_pu1;
  }
  if (speed_R1 < speed_ne1) {
    speed_R1 = speed_ne1;
  }
  //speed_L1 = abs(speed_L1);
  //  speed_R1 = abs(speed_R1);
  speed_LI1 = floor(speed_L1);
  speed_RI1 = floor(speed_R1);
  e_pre = e;
  if (e > 0 or e < 0) {
    RightAround3();
  }
  else {
    Forward();
  }
}

/**
 * @brief PID control for forward movement (variant 1).
 */
void PIDF1() {
  e = relative_yaw - relative_yaw1;
  //  e1 = abs(e);
  control1 = kp1 * e + kd1 * (e - e_pre);
  speed_L1 = speed_n1 + control1;
  speed_R1 = speed_n1 - control1;
  if (speed_L1 > speed_pu1) {
    speed_L1 = speed_pu1;
  }
  if (speed_L1 < speed_ne1) {
    speed_L1 = speed_ne1;
  }
  if (speed_R1 > speed_pu1) {
    speed_R1 = speed_pu1;
  }
  if (speed_R1 < speed_ne1) {
    speed_R1 = speed_ne1;
  }
  //speed_L1 = abs(speed_L1);
  //  speed_R1 = abs(speed_R1);
  speed_LI1 = floor(speed_L1);
  speed_RI1 = floor(speed_R1);
  e_pre = e;
  if (e > 0 or e < 0) {
    RightAround3();
  }
  else {
    Forward();
  }
}

/**
 * @brief PID control for forward movement (variant 2).
 */
void PIDF2() {
  e = relative_yaw - relative_yaw1;
  // e1 = abs(e);
  control1 = kp1 * e + kd1 * (e - e_pre);
  speed_L1 = speed_n1 + control1;
  speed_R1 = speed_n1 - control1;
  if (speed_L1 > speed_pu1) {
    speed_L1 = speed_pu1;
  }
  if (speed_L1 < speed_ne1) {
    speed_L1 = speed_ne1;
  }
  if (speed_R1 > speed_pu1) {
    speed_R1 = speed_pu1;
  }
  if (speed_R1 < speed_ne1) {
    speed_R1 = speed_ne1;
  }
  //speed_L1 = abs(speed_L1);
  //  speed_R1 = abs(speed_R1);
  speed_LI1 = floor(speed_L1);
  speed_RI1 = floor(speed_R1);
  e_pre = e;
  if (e > 0 or e < 0) {
    RightAround3();
  }
  else {
    Forward();
  }
}

/**
 * @brief PID control for backward movement.
 */
void PIDB() {
  e = relative_yaw - relative_yaw1;
  // e1 = abs(e);
  control1 = kp1 * e + kd1 * (e - e_pre);
  speed_L1 = 120 + control1;
  speed_R1 = 120 - control1;
  if (speed_L1 > 130) {
    speed_L1 = 130;
  }
  if (speed_L1 < 110) {
    speed_L1 = 110;
  }
  if (speed_R1 > 130) {
    speed_R1 = 130;
  }
  if (speed_R1 < 110) {
    speed_R1 = 110;
  }
  //speed_L1 = abs(speed_L1);
  //  speed_R1 = abs(speed_R1);
  speed_LI1 = floor(speed_L1);
  speed_RI1 = floor(speed_R1);
  e_pre = e;
  if (e > 0 or e < 0) {
    LeftAround3();
  }
  else {
    Backward();
  }
}

/**
 * @brief Rotates robot right.
 */
void RightAround() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);
  analogWrite(en1, speed_RI);
  analogWrite(en3, speed_RI);
  analogWrite(en2, speed_LI);
  analogWrite(en4, speed_LI);
}

/**
 * @brief Rotates robot left.
 */
void LeftAround() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
  analogWrite(en1, speed_LI);
  analogWrite(en3, speed_LI);
  analogWrite(en2, speed_RI);
  analogWrite(en4, speed_RI);
}

/**
 * @brief Moves robot forward-right (diagonal/strafe).
 */
void ForwardAround() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
  analogWrite(en1, speed_RI1);
  analogWrite(en4, speed_RI1);
  analogWrite(en2, speed_LI1);
  analogWrite(en3, speed_LI1);
}

/**
 * @brief Moves robot backward-left (diagonal/strafe).
 */
void BackAround() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
  analogWrite(en1, speed_LI1);
  analogWrite(en4, speed_LI1);
  analogWrite(en2, speed_RI1);
  analogWrite(en3, speed_RI1);
}

/**
 * @brief Moves robot rightward.
 */
void Rightward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);
  analogWrite(en1, speed_n3);
  analogWrite(en2, speed_n3);
  analogWrite(en3, speed_n3);
  analogWrite(en4, speed_n3);
}

/**
 * @brief Moves robot rightward (speed 2).
 */
void Rightward2() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);
  analogWrite(en1, speed_n2);
  analogWrite(en2, speed_n2);
  analogWrite(en3, speed_n2);
  analogWrite(en4, speed_n2);
}

/**
 * @brief Rotates robot right (variant 1).
 */
void RightAround1() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);
  analogWrite(en1, 70);
  analogWrite(en3, 70);
  analogWrite(en2, 70);
  analogWrite(en4, 70);
}

/**
 * @brief Rotates robot left (variant 1).
 */
void LeftAround1() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
  analogWrite(en1, 70);
  analogWrite(en3, 70);
  analogWrite(en2, 70);
  analogWrite(en4, 70);
}

/**
 * @brief Rotates robot right (variant 2).
 */
void RightAround2() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);
  analogWrite(en1, 60);
  analogWrite(en3, 60);
  analogWrite(en2, 60);
  analogWrite(en4, 60);
}

/**
 * @brief Rotates robot left (variant 2).
 */
void LeftAround2() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
  analogWrite(en1, 60);
  analogWrite(en3, 60);
  analogWrite(en2, 60);
  analogWrite(en4, 60);
}

/**
 * @brief Rotates robot right (variant 3).
 */
void RightAround3() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
  analogWrite(en1, speed_LI1);
  analogWrite(en2, speed_LI1);
  analogWrite(en3, speed_RI1);
  analogWrite(en4, speed_RI1);
}

/**
 * @brief Moves robot leftward.
 */
void Leftward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
  analogWrite(en1, speed_n3);
  analogWrite(en2, speed_n3);
  analogWrite(en3, speed_n3);
  analogWrite(en4, speed_n3);
}

/**
 * @brief Moves robot leftward (variant 1).
 */
void Leftward1() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
  analogWrite(en1, 60);
  analogWrite(en2, 60);
  analogWrite(en3, 60);
  analogWrite(en4, 60);
}

/**
 * @brief Moves robot leftward (variant 2).
 */
void Leftward2() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
  analogWrite(en1, speed_n2);
  analogWrite(en2, speed_n2);
  analogWrite(en3, speed_n2);
  analogWrite(en4, speed_n2);
}

/**
 * @brief Rotates robot left (variant 3).
 */
void LeftAround3() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);
  analogWrite(en1, speed_RI1);
  analogWrite(en2, speed_RI1);
  analogWrite(en3, speed_LI1);
  analogWrite(en4, speed_LI1);
}

/**
 * @brief Moves robot forward.
 */
void Forward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
  analogWrite(en1, speed_n1);
  analogWrite(en2, speed_n1);
  analogWrite(en3, speed_n1);
  analogWrite(en4, speed_n1);
}

/**
 * @brief Moves robot backward.
 */
void Backward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);
  analogWrite(en1, 110);
  analogWrite(en2, 110);
  analogWrite(en3, 110);
  analogWrite(en4, 110);
}

/**
 * @brief Moves robot forward (variant 1).
 */
void Forward1() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
  analogWrite(en1, 120);
  analogWrite(en2, 120);
  analogWrite(en3, 120);
  analogWrite(en4, 120);
}

//---------------------
/**
 * @brief Stops all motors.
 */
void Motor_reset() {
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
 * @brief Configures a KS103 sensor.
 */
void setting_ks103(byte addr, byte command) {
  Wire.beginTransmission(addr);
  Wire.write(byte(0x02));
  Wire.write(command);   // 发送降噪指令
  Wire.endTransmission();
  delay(1000);
}

/**
 * @brief Updates distance readings from KS103 sensors.
 */
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
    lai1 = 0;
  }
}

//void servo_reset() {
//  digitalWrite(angle90, HIGH);
//  digitalWrite(angle180, HIGH);
//  digitalWrite(angle135, HIGH);
//}

/**
 * @brief Sets status indicator to Green.
 */
void led_green() {
  digitalWrite(Buzzer1, HIGH);
  digitalWrite(Buzzer2, LOW);
}

/**
 * @brief Sets status indicator to Red.
 */
void led_red() {
  digitalWrite(Buzzer1, LOW);
  digitalWrite(Buzzer2, HIGH);
}

/**
 * @brief Turns off status indicator.
 */
void led_off() {
  digitalWrite(Buzzer1, LOW);
  digitalWrite(Buzzer2, LOW);
}

/**
 * @brief Setup function.
 *
 * Initializes hardware and sensors.
 */
void setup() {
  pinMode(to_openmv,OUTPUT);
  digitalWrite(to_openmv,HIGH);
  pinMode(get_openmv,INPUT_PULLUP);

  pinMode(is_shot_pin, OUTPUT);
  digitalWrite(is_shot_pin, HIGH);
  pinMode(is_shot_plus_pin, OUTPUT);
  digitalWrite(is_shot_plus_pin, HIGH);

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

  pinMode(angle90, OUTPUT);
  //  pinMode(angle180, OUTPUT);
  //  pinMode(angle135, OUTPUT);

  //  pinMode(riseball_pin, OUTPUT);
  //  pinMode(sweepball_pin, OUTPUT);
  //  digitalWrite(riseball_pin, LOW);
  //  digitalWrite(sweepball_pin, LOW);

  pinMode(pixy_color_flag_pin, OUTPUT);
  digitalWrite(pixy_color_flag_pin, HIGH);
  //  pinMode(is_start_pin, OUTPUT);
  //  digitalWrite(is_start_pin, HIGH);

  pinMode(team_color_pin, INPUT_PULLUP);
  pinMode(start_bt_pin, INPUT_PULLUP);
  //  pinMode(is_no_ball, INPUT_PULLUP);
  pinMode(plus_ball_over_pin, INPUT_PULLUP);

  pinMode(Buzzer1, OUTPUT);
  pinMode(Buzzer2, OUTPUT);
  led_red();
  Wire.begin();
  //  digitalWrite(SDA,LOW);
  //  digitalWrite(SCL,LOW);
  setting_ks103(KS103_L, 0x75);
  setting_ks103(KS103_R, 0x75);
  // put your setup code here, to run once:
  //  Serial.begin(115200);
  //  Serial.println("start");
  //=============================
  uint8_t val;
  mpu.SetAddress(MPU6050_ADDRESS_AD0_LOW).CalibrateMPU().load_DMP_Image();// Does it all for you with Calibration
  mpu.on_FIFO(get_yaw);
  //=============================
  //  Motor_reset();

  if (digitalRead(team_color_pin) == LOW) {
    team_color = 1;//黃
  } else {
    team_color = 2;//橘
  }

  digitalWrite(angle90, HIGH);//HIGH 180 LOW 90

  led_green();
}


/**
 * @brief Main loop.
 *
 * Executes the main game logic state machine for testing.
 */
void loop() {

  if (run_step == false) {
    if (digitalRead(start_bt_pin) == HIGH) {
      flag = 0;
      mpu.dmp_read_fifo();
      relative_yaw1 = relative_yaw;
      run_step = true;
      //      digitalWrite(is_start_pin, LOW);
    }
  }

  //------sensor更新------

  mpu.dmp_read_fifo();
  ks103_update();
  //  ks103_update();
  //---------------------

  //=======debug=========
  //  Serial.print("relative_yaw:");
  //  Serial.println(relative_yaw);
  //
  //
  //  Serial.print("L:");
  //  Serial.print(distance_L);
  //  Serial.print("R:");
  //  Serial.println(distance_R);
  //  //Serial.print("Counter:");
  //  //Serial.println(lai2);
  //  if (digitalRead(2)==LOW) {
  //    lai3=0;
  //  }
  //  if (digitalRead(2)==HIGH) {
  //    lai3=1;
  //  }
  //  //Serial.print("NTERRUPT:");
  //  //Serial.println(lai3);
  //=====================
  if (team_color == 1) {
    yello_team();
  } else {
    orange_team();
  }
}

/**
 * @brief Strategy state machine for Yellow team (test version).
 */
void yello_team() {
  if (((distance_L > 250) or (distance_R >= 500 and STOP1 == 1)) and flag == 38) {
    PIDR2();
    pidtest_time = millis();
    STOP1 = 1;
  } else if (flag == 38) {
    if (millis() - pidtest_time < 1000) {
      Motor_reset();
      lai = 1;
      STOP1 = 0;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;
      e_pre = 0;
      digitalWrite(angle90, LOW);
      speed_n1 = 120;
      speed_ne1 = 110;
      speed_pu1 = 130;
    }
  }

}

/**
 * @brief Strategy state machine for Orange team (test version).
 */
void orange_team() {
  if (((distance_L < 220 and distance_L >= 130) or (distance_L >= 500 and STOP1 == 1)) and flag == 38) {
    PIDL2();
    pidtest_time = millis();
    STOP1 = 1;
  } else if (flag == 38) {
    if (millis() - pidtest_time < 1000) {
      Motor_reset();
      lai = 1;
      STOP1 = 0;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;
      e_pre = 0;
      digitalWrite(angle90, LOW);
      speed_n1 = 120;
      speed_ne1 = 110;
      speed_pu1 = 130;
    }
  }

}
