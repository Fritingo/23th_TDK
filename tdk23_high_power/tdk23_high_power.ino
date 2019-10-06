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
int f_time = 950;
int cross_f_time = 1750;
float relative_yaw_error = 0;
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
int kp = 0.8;
int kd = 0.5;
int kp1 = 0.8;
int kd1 = 0.5;
int speed_n = 100;
int speed_n3 = 70;
int speed_ne3 = -80;
int speed_pu3 = 80;
int speed_ne = -110;
int speed_pu = 110;
int speed_n2 = 50;
int speed_ne2 = -60;
int speed_pu2 = 60;
int speed_n1 = 95;
int speed_ne1 = -110;
int speed_pu1 = 110;
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
long lai2 = 0;
int lai3 = 0;

//---------func-----------
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

void PIDR1() {
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
  if (e > 1) {
    RightAround();
  }
  else if (e < -1) {
    LeftAround();
  }
  else {
    Rightward();
  }
}
void PIDR2() {
  e = relative_yaw - relative_yaw1;
  e1 = abs(e);
  control = kp * e1 + kd * (e1 - e_pre);
  speed_L = speed_n2 + control;
  // speed_R = speed_n - control;
  if (speed_L > speed_pu2) {
    speed_L = speed_pu2;
  }
  //  if (speed_L < speed_ne) {
  //    speed_L = speed_ne;
  //  }
  if (speed_R > speed_pu2) {
    speed_R = speed_pu2;
  }
  //  if (speed_R < speed_ne) {
  //    speed_R = speed_ne;
  //  }
  speed_L = abs(speed_L);
  //  speed_R = abs(speed_R);
  speed_LI = floor(speed_L);
  //  speed_RI = floor(speed_R);
  e_pre = e1;
  if (e > 1) {
    RightAround();
  }
  else if (e < -1) {
    LeftAround();
  }
  else {
    Rightward2();
  }
}
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

void PIDL1() {
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
  if (e > 1) {
    RightAround();
  }
  else if (e < -1) {
    LeftAround();
  }
  else {
    Leftward();
  }
}
void PIDL2() {
  e = relative_yaw - relative_yaw1;
  e1 = abs(e);
  control = kp * e1 + kd * (e1 - e_pre);
  speed_L = speed_n2 + control;
  //  speed_R = speed_n - control;
  if (speed_L > speed_pu2) {
    speed_L = speed_pu2;
  }
  //  if (speed_L < speed_ne) {
  //    speed_L = speed_ne;
  //  }
  if (speed_R > speed_pu2) {
    speed_R = speed_pu2;
  }
  //  if (speed_R < speed_ne) {
  //    speed_R = speed_ne;
  //  }
  speed_L = abs(speed_L);
  //  speed_R = abs(speed_R);
  speed_LI = floor(speed_L);
  //  speed_RI = floor(speed_R);
  e_pre = e1;
  if (e > 1) {
    RightAround();
  }
  else if (e < -1) {
    LeftAround();
  }
  else {
    Leftward2();
  }
}
void PIDF() {
  e = relative_yaw - relative_yaw1;
  e1 = abs(e);
  control1 = kp1 * e1 + kd1 * (e1 - e_pre);
  speed_L1 = speed_n1 + control1;
  //  speed_R1 = speed_n1 - control1;
  if (speed_L1 > speed_pu1) {
    speed_L1 = speed_pu1;
  }
  //  if (speed_L1 < speed_ne1) {
  //    speed_L1 = speed_ne1;
  //  }
  //  if (speed_R1 > speed_pu1) {
  //    speed_R1 = speed_pu1;
  //  }
  //  if (speed_R1 < speed_ne1) {
  //    speed_R1 = speed_ne1;
  //  }
  speed_L1 = abs(speed_L1);
  //  speed_R1 = abs(speed_R1);
  speed_LI = floor(speed_L1);
  //  speed_RI1 = floor(speed_R1);
  e_pre = e1;
  if (e > 2) {
    RightAround();
  }
  else if (e < -2) {
    LeftAround();
  }
  else {
    Forward();
  }
}

void PIDF1() {
  e = relative_yaw - relative_yaw1;
  e1 = abs(e);
  control1 = kp1 * e1 + kd1 * (e1 - e_pre);
  speed_L1 = speed_n1 + control1;
  //  speed_R1 = speed_n1 - control1;
  if (speed_L1 > speed_pu1) {
    speed_L1 = speed_pu1;
  }
  //  if (speed_L1 < speed_ne1) {
  //    speed_L1 = speed_ne1;
  //  }
  //  if (speed_R1 > speed_pu1) {
  //    speed_R1 = speed_pu1;
  //  }
  //  if (speed_R1 < speed_ne1) {
  //    speed_R1 = speed_ne1;
  //  }
  speed_L1 = abs(speed_L1);
  //  speed_R1 = abs(speed_R1);
  speed_LI = floor(speed_L1);
  //  speed_RI1 = floor(speed_R1);
  e_pre = e1;
  if (e > 1) {
    RightAround();
  }
  else if (e < -1) {
    LeftAround();
  }
  else {
    Forward();
  }
}

void PIDF2() {
  e = relative_yaw - relative_yaw1;
  e1 = abs(e);
  control1 = kp1 * e1 + kd1 * (e1 - e_pre);
  speed_L1 = speed_n1 + control1;
  //  speed_R1 = speed_n1 - control1;
  if (speed_L1 > speed_pu1) {
    speed_L1 = speed_pu1;
  }
  //  if (speed_L1 < speed_ne1) {
  //    speed_L1 = speed_ne1;
  //  }
  //  if (speed_R1 > speed_pu1) {
  //    speed_R1 = speed_pu1;
  //  }
  //  if (speed_R1 < speed_ne1) {
  //    speed_R1 = speed_ne1;
  //  }
  speed_L1 = abs(speed_L1);
  //  speed_R1 = abs(speed_R1);
  speed_LI = floor(speed_L1);
  //  speed_RI1 = floor(speed_R1);
  e_pre = e1;
  if (e > 0.8) {
    RightAround();
  }
  else if (e < -0.8) {
    LeftAround();
  }
  else {
    Forward();
  }
}

void RightAround() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);
  analogWrite(en1, speed_LI);
  analogWrite(en3, speed_LI);
  analogWrite(en2, speed_LI);
  analogWrite(en4, speed_LI);
}

void LeftAround() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
  analogWrite(en1, speed_LI);
  analogWrite(en3, speed_LI);
  analogWrite(en2, speed_LI);
  analogWrite(en4, speed_LI);
}

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

void Rightward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);
  analogWrite(en1, speed_n);
  analogWrite(en2, speed_n);
  analogWrite(en3, speed_n);
  analogWrite(en4, speed_n);
}
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
void RightAround1() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);
  analogWrite(en1, 80);
  analogWrite(en3, 80);
  analogWrite(en2, 80);
  analogWrite(en4, 80);
}

void LeftAround1() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
  analogWrite(en1, 80);
  analogWrite(en3, 80);
  analogWrite(en2, 80);
  analogWrite(en4, 80);
}
void RightAround2() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);
  analogWrite(en1, 55);
  analogWrite(en3, 55);
  analogWrite(en2, 55);
  analogWrite(en4, 55);
}

void LeftAround2() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
  analogWrite(en1, 55);
  analogWrite(en3, 55);
  analogWrite(en2, 55);
  analogWrite(en4, 55);
}
void Leftward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
  analogWrite(en1, speed_n);
  analogWrite(en2, speed_n);
  analogWrite(en3, speed_n);
  analogWrite(en4, speed_n);
}

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
void Forward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
  analogWrite(en1, 180);
  analogWrite(en2, 180);
  analogWrite(en3, 180);
  analogWrite(en4, 180);
}

void Forward1() {
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

//---------------------
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
    lai1 = 0;
  }
}

//void servo_reset() {
//  digitalWrite(angle90, HIGH);
//  digitalWrite(angle180, HIGH);
//  digitalWrite(angle135, HIGH);
//}

void led_green() {
  digitalWrite(Buzzer1, HIGH);
  digitalWrite(Buzzer2, LOW);
}

void led_red() {
  digitalWrite(Buzzer1, LOW);
  digitalWrite(Buzzer2, HIGH);
}

void led_off() {
  digitalWrite(Buzzer1, LOW);
  digitalWrite(Buzzer2, LOW);
}

void setup() {

  //  pinMode(is_shot_plus_pin, OUTPUT);
  //  digitalWrite(is_shot_plus_pin, HIGH);
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

void yello_team() {
  if (distance_L < 110 and flag == 0 and lai == 0 or distance_L == 519) {
    PIDL1();
    pidtest_time = millis();
  } else if (flag == 0) {
    if (millis() - pidtest_time < 800) {
      Motor_reset();
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;
      digitalWrite(angle90, LOW);
    }
  }

  if (flag == 1) {
    if (millis() - pidtest_time < 1900) {
      PIDF();
    } else {
      Motor_reset();
      flag++;

      digitalWrite(angle90, HIGH);
      pidtest_time = millis();
      digitalWrite(pixy_color_flag_pin, LOW);
    }
  }

  if (flag == 2) {
    if (millis() - pidtest_time < 3000) {
      Motor_reset();
    } else {
      flag++;

    }
  }

  if (distance_R > 75  and flag == 3 and lai == 0) {
    PIDL1();
    pidtest_time = millis();
  } else if (flag == 3) {
    if (millis() - pidtest_time < 800) {
      Motor_reset();
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      digitalWrite(angle90, LOW);
    }
  }

  if (flag == 4) {
    if (millis() - pidtest_time < f_time) {
      PIDF();
    } else {
      Motor_reset();
      flag++;

      digitalWrite(angle90, HIGH);
      //  delay(100);
      pidtest_time = millis();
    }
  }

  if (flag == 5) {
    if (millis() - pidtest_time < 5000) {
      PIDR1();
    } else {
      Motor_reset();
      flag++;

      digitalWrite(angle90, LOW);
      pidtest_time = millis();
    }
  }

  if (flag == 6) {
    if (millis() - pidtest_time < cross_f_time) {
      PIDF();
    } else {
      Motor_reset();
      flag++;

      digitalWrite(angle90, HIGH);
    }
  }

  if (distance_R > 75 and flag == 7 and lai == 0) {
    PIDL1();
    pidtest_time = millis();
  } else if (flag == 7) {
    if (millis() - pidtest_time < 800) {
      Motor_reset();
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      digitalWrite(angle90, LOW);
    }
  }

  if (flag == 8) {
    if (millis() - pidtest_time < f_time) {
      PIDF();
    } else {
      Motor_reset();
      flag++;

      digitalWrite(angle90, HIGH);
      pidtest_time = millis();
      //    delay(100);
    }
  }
  if (flag == 9) {
    if (millis() - pidtest_time < 5000) {
      PIDR1();
    } else {
      Motor_reset();
      flag++;

      digitalWrite(angle90, LOW);
      pidtest_time = millis();
    }
  }

  if (flag == 10) {
    if (millis() - pidtest_time < cross_f_time) {
      PIDF();
    } else {
      Motor_reset();
      flag++;

      digitalWrite(angle90, HIGH);
    }
  }

  if (distance_R > 75 and flag == 11 and lai == 0) {
    PIDL1();
    pidtest_time = millis();
  } else if (flag == 11) {
    if (millis() - pidtest_time < 800) {
      Motor_reset();
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      digitalWrite(angle90, LOW);
    }
  }

  if (flag == 12) {
    if (millis() - pidtest_time < f_time) {
      PIDF();
    } else {
      Motor_reset();
      flag++;

      digitalWrite(angle90, HIGH);
      //    delay(100);
    }
  }

  if (distance_L > 75 and flag == 13 and lai == 0) {
    PIDR1();
    pidtest_time = millis();
  } else if (flag == 13) {
    if (millis() - pidtest_time < 800) {
      Motor_reset();
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      digitalWrite(angle90, LOW);

    }
  }

  if (flag == 14) {
    if (millis() - pidtest_time < f_time) {
      PIDF();
    } else {
      Motor_reset();
      flag++;

      digitalWrite(angle90, HIGH);
    }
  }

  if (distance_R > 75 and flag == 15 and lai == 0) {
    PIDL1();
    pidtest_time = millis();
  } else if (flag == 15) {
    if (millis() - pidtest_time < 800) {
      Motor_reset();
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      digitalWrite(angle90, LOW);
    }
  }

  if (flag == 16) {
    if (millis() - pidtest_time < f_time) {
      PIDF();
    } else {
      Motor_reset();
      flag++;

      digitalWrite(angle90, HIGH);
    }
  }

  if (distance_L > 75 and flag == 17 and lai == 0) {
    PIDR1();
    pidtest_time = millis();
  } else if (flag == 17) {
    if (millis() - pidtest_time < 800) {
      Motor_reset();
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      digitalWrite(angle90, LOW);
    }
  }

  if (flag == 18) {
    if (millis() - pidtest_time < f_time) {
      PIDF();
    } else {
      Motor_reset();
      flag++;

      digitalWrite(angle90, HIGH);
    }
  }

  if (distance_R > 75 and flag == 19 and lai == 0) {
    PIDL1();
    pidtest_time = millis();
  } else if (flag == 19) {
    if (millis() - pidtest_time < 800) {
      Motor_reset();
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      digitalWrite(angle90, LOW);
    }
  }

  if (flag == 20) {
    if (millis() - pidtest_time < f_time) {
      PIDF();
    } else {
      Motor_reset();
      flag++;

      digitalWrite(angle90, HIGH);
    }
  }

  if (distance_L > 75 and flag == 21 and lai == 0) {
    PIDR1();
    pidtest_time = millis();
  } else if (flag == 21) {
    if (millis() - pidtest_time < 800) {
      Motor_reset();
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      digitalWrite(angle90, LOW);
    }
  }

  if (flag == 22) {
    if (millis() - pidtest_time < f_time) {
      PIDF();

    } else {
      Motor_reset();
      flag++;
      pidtest_time = millis();

      digitalWrite(angle90, HIGH);
    }
  }
  //---------------------------
  if (distance_R > 75 and flag == 23 and lai == 0) {
    PIDL1();
    pidtest_time = millis();
  } else if (flag == 23) {
    if (millis() - pidtest_time < 800) {
      Motor_reset();
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      digitalWrite(angle90, LOW);
    }
  }

  if (flag == 24) {
    if (millis() - pidtest_time < f_time) {
      PIDF();
    } else {
      Motor_reset();
      flag++;

      digitalWrite(angle90, HIGH);
    }
  }

  if (distance_L > 75 and flag == 25 and lai == 0) {
    PIDR1();
    pidtest_time = millis();
  } else if (flag == 25) {
    if (millis() - pidtest_time < 800) {
      Motor_reset();
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      digitalWrite(angle90, LOW);
    }
  }

  if (flag == 26) {
    if (millis() - pidtest_time < f_time) {
      PIDF();
    } else {
      Motor_reset();
      flag++;
      pidtest_time = millis();

      digitalWrite(angle90, HIGH);
    }
  }
  if (distance_R > 75  and flag == 27 and lai == 0) {
    PIDL1();
    pidtest_time = millis();
  } else if (flag == 27) {
    if (millis() - pidtest_time < 800) {
      Motor_reset();
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      digitalWrite(angle90, LOW);
    }
  }

  if (flag == 28) {
    if (millis() - pidtest_time < f_time) {
      PIDF();
    } else {
      Motor_reset();
      flag++;

      digitalWrite(angle90, HIGH);

    }
  }

  if (distance_L > 75 and flag == 29 and lai == 0) {
    PIDR1();
    pidtest_time = millis();
  } else if (flag == 29) {
    if (millis() - pidtest_time < 800) {
      Motor_reset();
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      digitalWrite(angle90, LOW);
    }
  }

  if (flag == 30) {
    if (millis() - pidtest_time < f_time) {
      PIDF();
    } else {
      Motor_reset();
      flag++;

      digitalWrite(angle90, HIGH);
    }
  }


  //--------find_plus----------
  if (flag == 31) {
    if (millis() - pidtest_time < 1000) {
      Motor_reset();
    }
    else {
      digitalWrite(angle90, HIGH);
      flag++;
    }
  }
  //===============flag change==========
  if (distance_R > 260 and flag == 32 and lai == 0)
  {
    PIDL1();
    pidtest_time = millis();
  } else if (distance_R > 130 and distance_R <= 260 and flag == 32 and lai == 0)
  {
    PIDL2();
    pidtest_time = millis();
  } else if (flag == 32) {
    if (millis() - pidtest_time < 1000) {
      Motor_reset();
      lai = 1;
      digitalWrite(angle90, LOW);
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      digitalWrite(angle90, LOW);
      speed_n1 = 80;
      speed_ne1 = -90;
      speed_pu1 = 90;
    }
  }

  if (distance_L > 165 and flag == 33 and lai == 0) {
    PIDF1();
    pidtest_time = millis();
  }else if (flag == 33) {
    if (millis() - pidtest_time < 1000) {
      Motor_reset();
      lai = 1;
    } else {
      if (distance_L - distance_R > 3) {
        lai = 1;
        LeftAround2();
      } else if (distance_R - distance_L > 3) {
        lai = 1;
        RightAround2();
      } else {
        //    relative_yaw_error = relative_yaw;
        Motor_reset();
        flag++;
        pidtest_time = millis();
        lai = 0;
        mpu.dmp_read_fifo();
        relative_yaw2 = relative_yaw;
        digitalWrite(angle90, LOW);
        //      if (digitalRead(is_no_ball) == LOW) {
        //
        //        digitalWrite(angle90, HIGH);
        //        flag = 32;
        //      } else {
        digitalWrite(is_shot_plus_pin, LOW);//射球
        relative_yaw1 = relative_yaw1 - relative_yaw2;
      }

    }
  }

  if (relative_yaw- relative_yaw2 < 42 and flag == 34 and lai == 0) {
    LeftAround1();
    pidtest_time = millis();
  } else if (flag == 34) {// and
    if (millis() - pidtest_time < 16000 and digitalRead(plus_ball_over_pin) == HIGH) {// or digitalRead(plus_ball_over_pin) == LOW
      Motor_reset();
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;
      //
      //  digitalWrite(angle90, LOW);
    }
  }

  if (relative_yaw- relative_yaw2 > 0.5 and flag == 35 and lai == 0) {
    RightAround1();
    pidtest_time = millis();
  } else if (flag == 35) {
    if (millis() - pidtest_time < 1000) {
      Motor_reset();
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      digitalWrite(angle90, HIGH);
    }
  }

  if (flag == 36) {
    if (millis() - pidtest_time < 1000) {
      Motor_reset();
    }
    else {
      flag++;
    }
  }

  if (distance_R < 150 and flag == 37 and lai == 0 or distance_R == 519)
  {
    PIDR1();
    //  pidtest_time = millis();
  } else if ((distance_L > 280 and distance_R >= 150 and flag == 37 and lai == 0) or distance_R == 519) {
    PIDR2();
    pidtest_time = millis();
  } else if (flag == 37) {
    if (millis() - pidtest_time < 1000) {
      Motor_reset();
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      digitalWrite(angle90, LOW);
      speed_n1 = 70;
      speed_ne1 = -80;
      speed_pu1 = 80;
    }
  }

  if (distance_R > 68 and flag == 38 and lai == 0) {
    PIDF1();
    pidtest_time = millis();
  } else if (flag == 38) {
    if (millis() - pidtest_time < 1000) {
      Motor_reset();
      lai = 1;
    } else if (distance_L - distance_R > 3) {
      lai = 1;
      LeftAround2();
    }else if (distance_R - distance_L > 3) {
      lai = 1;
      RightAround2();
    } else {
      Motor_reset();
      flag++;
      pidtest_time = millis();
      lai = 0;

      digitalWrite(angle90, LOW);
      digitalWrite(is_shot_pin, LOW);//射球
    }
  }

}

void orange_team() {
  if (distance_R < 150 and flag == 0 and lai == 0) {
    PIDR1();
    pidtest_time = millis();
  } else if (flag == 0) {
    if (millis() - pidtest_time < 1000) {
      Motor_reset();
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;
      digitalWrite(angle90, LOW);
    }
  }

  if (flag == 1) {
    if (millis() - pidtest_time < 1900) {
      PIDF();
    } else {
      Motor_reset();
      flag++;

      digitalWrite(angle90, HIGH);
      pidtest_time = millis();
      digitalWrite(pixy_color_flag_pin, LOW);
    }
  }

  if (flag == 2) {
    if (millis() - pidtest_time < 3000) {
      Motor_reset();
    } else {
      flag++;

    }
  }

  if (distance_L > 75 and flag == 3 and lai == 0) {
    PIDR1();
    pidtest_time = millis();
  } else if (flag == 3) {
    if (millis() - pidtest_time < 800) {
      Motor_reset();
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      digitalWrite(angle90, LOW);
    }
  }

  if (flag == 4) {
    if (millis() - pidtest_time < f_time) {
      PIDF();
    } else {
      Motor_reset();
      flag++;

      digitalWrite(angle90, HIGH);

    }
  }

  if (flag == 5) {
    if (millis() - pidtest_time < 5000) {
      PIDL1();
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;
      digitalWrite(angle90, LOW);
    }
  }

  if (flag == 6) {
    if (millis() - pidtest_time < cross_f_time) {
      PIDF();
    } else {
      Motor_reset();
      flag++;
      digitalWrite(angle90, HIGH);
    }
  }

  if (distance_L > 75 and flag == 7 and lai == 0) {
    PIDR1();
    pidtest_time = millis();
  } else if (flag == 7) {
    if (millis() - pidtest_time < 800) {
      Motor_reset();
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      digitalWrite(angle90, LOW);
    }
  }

  if (flag == 8) {
    if (millis() - pidtest_time < f_time) {
      PIDF();
    } else {
      Motor_reset();
      flag++;

      digitalWrite(angle90, HIGH);
    }
  }

  if (flag == 9) {
    if (millis() - pidtest_time < 5000) {
      PIDL1();
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;
      digitalWrite(angle90, LOW);
    }
  }

  if (flag == 10) {
    if (millis() - pidtest_time < cross_f_time) {
      PIDF();
    } else {
      Motor_reset();
      flag++;

      digitalWrite(angle90, HIGH);
    }
  }

  if (distance_L > 75 and flag == 11 and lai == 0) {
    PIDR1();
    pidtest_time = millis();
  } else if (flag == 11) {
    if (millis() - pidtest_time < 800) {
      Motor_reset();
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      digitalWrite(angle90, LOW);
    }
  }

  if (flag == 12) {
    if (millis() - pidtest_time < f_time) {
      PIDF();
    } else {
      Motor_reset();
      flag++;

      digitalWrite(angle90, HIGH);
    }
  }

  if (distance_R > 75 and flag == 13 and lai == 0) {
    PIDL1();
    pidtest_time = millis();
  } else if (flag == 13) {
    if (millis() - pidtest_time < 800) {
      Motor_reset();
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      digitalWrite(angle90, LOW);
    }
  }

  if (flag == 14) {
    if (millis() - pidtest_time < f_time) {
      PIDF();
    } else {
      Motor_reset();
      flag++;

      digitalWrite(angle90, HIGH);
    }
  }

  if (distance_L > 75 and flag == 15 and lai == 0) {
    PIDR1();
    pidtest_time = millis();
  } else if (flag == 15) {
    if (millis() - pidtest_time < 800) {
      Motor_reset();
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      digitalWrite(angle90, LOW);
    }
  }

  if (flag == 16) {
    if (millis() - pidtest_time < 1700) {
      PIDF();
    } else {
      Motor_reset();
      flag++;

      digitalWrite(angle90, HIGH);
    }
  }

  if (distance_R > 75 and flag == 17 and lai == 0) {
    PIDL1();
    pidtest_time = millis();
  } else if (flag == 17) {
    if (millis() - pidtest_time < 800) {
      Motor_reset();
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      digitalWrite(angle90, LOW);
    }
  }

  if (flag == 18) {
    if (millis() - pidtest_time < f_time) {
      PIDF();
    } else {
      Motor_reset();
      flag++;

      digitalWrite(angle90, HIGH);
    }
  }

  if (distance_L > 75 and flag == 19 and lai == 0) {
    PIDR1();
    pidtest_time = millis();
  } else if (flag == 19) {
    if (millis() - pidtest_time < 800) {
      Motor_reset();
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      digitalWrite(angle90, LOW);
    }
  }

  if (flag == 20) {
    if (millis() - pidtest_time < f_time) {
      PIDF();
    } else {
      Motor_reset();
      flag++;

      digitalWrite(angle90, HIGH);
    }
  }

  if (distance_R > 75 and flag == 21 and lai == 0) {
    PIDL1();
    pidtest_time = millis();
  } else if (flag == 21) {
    if (millis() - pidtest_time < 800) {
      Motor_reset();
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      digitalWrite(angle90, LOW);
    }
  }

  if (flag == 22) {
    if (millis() - pidtest_time < f_time) {
      PIDF();

    } else {
      Motor_reset();
      flag++;
      pidtest_time = millis();

      digitalWrite(angle90, HIGH);
    }
  }
  //---------------------------
  if (distance_L > 75 and flag == 23 and lai == 0) {
    PIDR1();
    pidtest_time = millis();
  } else if (flag == 23) {
    if (millis() - pidtest_time < 800) {
      Motor_reset();
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      digitalWrite(angle90, LOW);
    }
  }

  if (flag == 24) {
    if (millis() - pidtest_time < f_time) {
      PIDF();
    } else {
      Motor_reset();
      flag++;

      digitalWrite(angle90, HIGH);
    }
  }

  if (distance_R > 75 and flag == 25 and lai == 0) {
    PIDL1();
    pidtest_time = millis();
  } else if (flag == 25) {
    if (millis() - pidtest_time < 800) {
      Motor_reset();
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      digitalWrite(angle90, LOW);
    }
  }

  if (flag == 26) {
    if (millis() - pidtest_time < f_time) {
      PIDF();
    } else {
      Motor_reset();
      flag++;
      pidtest_time = millis();

      digitalWrite(angle90, HIGH);
    }
  }


  //--------find_plus----------
  if (flag == 27) {
    if (millis() - pidtest_time < 800) {
      Motor_reset();
    }
    else {
      digitalWrite(angle90, HIGH);
      flag++;
    }
  }

  if (distance_L > 75 and flag == 28 and lai == 0) {
    PIDR1();
    pidtest_time = millis();
  } else if (flag == 28) {
    if (millis() - pidtest_time < 800) {
      Motor_reset();
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      digitalWrite(angle90, LOW);
    }
  }

  if (flag == 29) {
    if (millis() - pidtest_time < f_time) {
      PIDF();
    } else {
      Motor_reset();
      flag++;

      digitalWrite(angle90, HIGH);

    }
  }

  if (distance_R > 75 and flag == 30 and lai == 0) {
    PIDL1();
    pidtest_time = millis();
  } else if (flag == 30) {
    if (millis() - pidtest_time < 800) {
      Motor_reset();
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      digitalWrite(angle90, LOW);
    }
  }

  if (flag == 31) {
    if (millis() - pidtest_time < f_time) {
      PIDF();
    } else {
      Motor_reset();
      flag++;

      digitalWrite(angle90, HIGH);

    }
  }
  //===============flag change==========
  if (distance_L > 250 and flag == 32 and lai == 0)
  {
    PIDR1();
    pidtest_time = millis();
  } else if (distance_L > 130 and distance_L <= 250 and flag == 32 and lai == 0) {
    PIDR2();
    pidtest_time = millis();
  } else if (flag == 32) {
    if (millis() - pidtest_time < 1000) {
      Motor_reset();
      lai = 1;
      digitalWrite(angle90, LOW);
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      digitalWrite(angle90, LOW);
      speed_n1 = 80;
      speed_ne1 = -90;
      speed_pu1 = 90;
    }
  }

  if (distance_R > 160 and flag == 33 and lai == 0) {
    PIDF1();
    pidtest_time = millis();
  } else if (flag == 33) {
    if (millis() - pidtest_time < 1000) {
      Motor_reset();
      lai = 1;
    } else {
      if (distance_L - distance_R > 3) {
        lai = 1;
        LeftAround2();
      } else if (distance_R - distance_L > 3) {
        lai = 1;
        RightAround2();
      } else {
        //    relative_yaw_error = relative_yaw;
        Motor_reset();
        flag++;
        pidtest_time = millis();
        lai = 0;
        mpu.dmp_read_fifo();
        relative_yaw2 = relative_yaw;
        digitalWrite(angle90, LOW);
        //      if (digitalRead(is_no_ball) == LOW) {
        //
        //        digitalWrite(angle90, HIGH);
        //        flag = 32;
        //      } else {
        digitalWrite(is_shot_plus_pin, LOW);//射球
        relative_yaw1 = relative_yaw1 - relative_yaw2;
        //      }
      }
    }
  }

  if (relative_yaw - relative_yaw2 > -47 and flag == 34 and lai == 0) {
    RightAround1();
    pidtest_time = millis();
  } else if (flag == 34) {// and
    if (millis() - pidtest_time < 16000 and digitalRead(plus_ball_over_pin) == HIGH) {// or digitalRead(plus_ball_over_pin) == LOW
      Motor_reset();
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;
      //
      //  digitalWrite(angle90, LOW);
    }
  }

  if (relative_yaw - relative_yaw2 < -1 and flag == 35 and lai == 0) {
    LeftAround1();
    pidtest_time = millis();
  } else if (flag == 35) {
    if (millis() - pidtest_time < 1000) {
      Motor_reset();
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      digitalWrite(angle90, HIGH);
    }
  }

  if (flag == 36) {
    if (millis() - pidtest_time < 1000) {
      Motor_reset();
    }
    else {
      flag++;

    }
  }

  if (distance_L < 130 and flag == 37 and lai == 0)
  {
    PIDL1();
    pidtest_time = millis();
  } else if (distance_R > 305 and distance_L >= 130 and flag == 37 and lai == 0) {
    PIDL2();
    pidtest_time = millis();
  } else if (flag == 37) {
    if (millis() - pidtest_time < 1000) {
      Motor_reset();
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;

      digitalWrite(angle90, LOW);
      speed_n1 = 70;
      speed_ne1 = -80;
      speed_pu1 = 80;
    }
  }

  if (distance_R > 68 and flag == 38 and lai == 0) {
    PIDF1();
    pidtest_time = millis();
  } else if (flag == 38) {
    if (millis() - pidtest_time < 1000) {
      Motor_reset();
      lai = 1;
    } else if (distance_L - distance_R > 3) {
      lai = 1;
      LeftAround2();
    }
    else if (distance_R - distance_L > 3) {
      lai = 1;
      RightAround2();
    } else {
      Motor_reset();
      flag++;
      pidtest_time = millis();
      lai = 0;

      digitalWrite(angle90, LOW);
      digitalWrite(is_shot_pin, LOW);//射球
    }
  }

}
