#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <NewPing.h>
#include "Wire.h"
#define Pattern 'R'//A,AUTO;R,ROMOTE

bool Start = false;
bool original_start = false;
int STEP = 0;
int sweep_ball_step = 0;
int step_move = 0;
const int IR_turns_sensor = 45;
int turn_total = 0;
int turn_count = 0;
int pre_IR_turn_val = 1;
int IR_turn_val;
bool is_mode;
int mode_code = 0;

bool is_Move_start = false;
bool is_correction_angle = false;

float distance_x, distance_y;
float distance_R, distance_L;

#if Pattern == 'A'
#include <Servo.h>

Servo sonic_servoR;
Servo sonic_servoL;

void Move(char Direction, int Turns, int sonic_Distance, int Speed) { //方向(前F後B左L右R),輪胎圈數,超音波距離,輪胎速度
  if (is_Move_start == false) {                                   //向(方向)用(輪胎速度)轉(輪胎圈數)後用低速調整至(超音波距離)
    is_Move_start = true;
    is_correction_angle = false;
    turn_count = turn_total;
    switch (Direction) {
      case 'F'://F
        sonic_servoR.write(90);
        break;
      case 'B'://B
        sonic_servoL.write(90);
        break;
      case 'L'://L
        sonic_servoL.write(0);
        sonic_servoR.write(90);
        break;
      case 'R'://R
        sonic_servoL.write(90);
        sonic_servoR.write(180);
        break;
      case '3'://LF
        sonic_servoR.write(45);
        break;
      case '1'://RF
        sonic_servoR.write(135);
        break;
      case '4'://LB
        sonic_servoR.write(45);
        break;
      case '2'://RB
        sonic_servoR.write(135);
        break;
    }
  } else {
    if (turn_total - turn_count < Turns) {
      switch (Direction) {
        case 'F'://F
          m_type_Forward(Speed);
          break;
        case 'B'://B
          m_type_Backward(Speed);
          break;
        case 'L'://L
          m_type_Leftward(Speed);
          break;
        case 'R'://R
          m_type_Rightward(Speed);
          break;
        case '3'://LF
          m_type_LeftForward(Speed);
          break;
        case '1'://RF
          m_type_RightForward(Speed);
          break;
        case '4'://LB
          m_type_LeftBackward(Speed);
          break;
        case '2'://RB
          m_type_RightBackward(Speed);
          break;
      }
    } else {
      if (((turn_total - turn_count) - Turns) < 5) {
        if ((Speed - (turn_total - turn_count) * 2) < 50) {//移動最低速
          Motor_directly(50);
        } else {
          Motor_directly(Speed - (turn_total - turn_count) * 2);
        }
      } else {
        switch (Direction) {
          case 'F'://F
            if (is_correction_angle == true) {
              if (int(distance_L) <= 60) { //進入發球距離
                STEP++;
              } else {
                is_Move_start = false;
                sweep_ball_step++;
              }
            } else {
              m_type_correction_angle();
            }
            break;
          case 'B'://B
            if (is_correction_angle == true) {
              is_Move_start = false;
              sweep_ball_step++;
            } else {
              m_type_correction_angle();
            }
            break;
          case 'L'://L
            if (int(distance_L) <= sonic_Distance) {
              if (is_correction_angle == true) {
                is_Move_start = false;
                sweep_ball_step++;
              } else {
                m_type_correction_angle();
              }
            }
            break;
          case 'R'://R
            if (int(distance_R) <= sonic_Distance) {
              if (is_correction_angle == true) {
                is_Move_start = false;
                sweep_ball_step++;
              } else {
                m_type_correction_angle();
              }
            }
            break;
          case '3'://"LF"
            if (int(distance_L) <= sonic_Distance) {
              is_Move_start = false;
              STEP++;
            }
            break;
          case '1'://"RF"
            if (int(distance_R) <= sonic_Distance) {
              is_Move_start = false;
              STEP++;
              break;
            }
          case '4'://LB
            if (int(distance_L) <= sonic_Distance) {
              is_Move_start = false;
              STEP++;
            }
            break;
          case '2'://RB
            if (int(distance_R) <= sonic_Distance) {
              is_Move_start = false;
              STEP++;
            }
        }
      }
    }
  }
}

#endif

#if Pattern == 'R'
#include <IRremote.h>

IRrecv irrecv(11); // 使用數位腳位11接收紅外線訊號初始化紅外線訊號輸入
decode_results results; // 儲存訊號的結構

void IR_update() {
  if (irrecv.decode(&results)) { // 接收紅外線訊號並解碼
    Serial.print("results value is "); // 輸出解碼後的資料//0:16738455/1:16724175/2:16718055/3:16743045/4:16716015/5:16726215/6:16734885/7:16728765/8:16730805/9:16732845
    Serial.println(results.value);//0:FF6897/1:FF6897/2:FF18E7/3:FF7A85/4:FF10EF/5:FF38C7/6:FF5AA5/7:FF42BD/8:FF4AB5/9:FF52AD
    is_mode = results.value == 16738455 ||
              results.value == 16724175 ||
              results.value == 16718055 ||
              results.value == 16743045 ||
              results.value == 16716015 ||
              results.value == 16726215 ||
              results.value == 16734885 ||
              results.value == 16728765 ||
              results.value == 16730805 ||
              results.value == 16732845;
    if (is_mode) {
      mode_code = results.value;
    }
    irrecv.resume(); // 準備接收下一個訊號
  }
}

#endif


MPU6050 mpu;



const int start_bt = 23;




#define SONAR_NUM     2 // Number of sensors.
#define MAX_DISTANCE 400 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
float cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.


NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(51, 53, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(47, 49, MAX_DISTANCE)
};

unsigned long sonar_start = 0;
// 定策略時要修改的距離
#define d1 80
#define d2 150
#define d3 55
#define d5 150
#define car_y 69
#define stop_collect_y 20
int loop_x = d5;
int loop_y = d2 - (2 * car_y);

unsigned long step_start;
unsigned long initial;
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
const int en3 = 6;
const int en4 = 5;

const int Rsonic_servo = 7;
const int Lsonic_servo = 8;

const int Buzzer = 39;

float original_z = 0;
bool gyro_ready = false;


int count = 0;
bool is_start = true;
bool is_end = false;
int activate_speed;
bool is_started = false;
bool is_brake = false;
unsigned long stop_x = 0;
unsigned long stop_y = 0;
unsigned long motor_start = 0;
float relative_yaw;
float base_yaw;
float goal_yaw;

#define INTERRUPT_PIN 19  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yaw = -1.0;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
  mpuInterrupt = true;
}

void mpu6050_setup() {

  float first_yaw = 0;
  int counter_ready = 0;
  float last_yaw = 999;

  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

    while (!mpuInterrupt && fifoCount < packetSize) {
      if (mpuInterrupt && fifoCount < packetSize) {
        // try to get out of the infinite loop
        fifoCount = mpu.getFIFOCount();
      }
    }
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  if (dmpReady) {
    Serial.println("進行陀螺儀校準中...");
    do {
      if (mpuInterrupt) {
        if (fifoCount < packetSize) {
          fifoCount = mpu.getFIFOCount();
        } else {
          mpu6050_update();
          //          Serial.println(yaw);
          if (counter_ready < 100) {
            if (counter_ready == 0) {
              first_yaw = yaw;
            } else if (counter_ready == 99) {
              last_yaw = yaw;
            }
            counter_ready++;
          } else {
            Serial.print("校準差值 ");
            Serial.print(last_yaw - first_yaw);
            Serial.println(" 未小於0.05");
            counter_ready = 0;
          }
        }
      }
    } while (!(counter_ready >= 100 && (abs(last_yaw - first_yaw) < 0.05)));
    gyro_ready = true;
    original_z = yaw;
    Serial.println("進入策略模式");
    digitalWrite(Buzzer, LOW);
  } else {
    // if programming failed, don't try to do anything
    Serial.println("陀螺儀初始化錯誤!!!");
  }
}

void mpu6050_update() {
  if (mpuInterrupt) {
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      fifoCount = mpu.getFIFOCount();
      Serial.println(F("FIFO overflow!"));

      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      mpu.resetFIFO();
      fifoCount = 0;
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      //    fifoCount -= packetSize;
      // update Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      yaw = (ypr[0] * 180 / M_PI);
      relative_yaw = yaw - original_z;
      if (relative_yaw > 180) {
        relative_yaw = relative_yaw - 360;
      } else if (relative_yaw < -180) {
        relative_yaw = relative_yaw + 360;
      }
      //            Serial.print("yaw = ");
      //            Serial.println(relative_yaw);
    }
  }
}

bool mpu6050_getyaw() {
  if (!mpuInterrupt && fifoCount < packetSize) {
    return true;
  } else {
    mpu6050_update();
    return false;
  }
}

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

void Motor_start(int Speed) {
  if (!is_end) {
    if (Speed < 50) {
      activate_speed = 50;
    } else {
      activate_speed = Speed;
    }
    if (count < (activate_speed / 10)) {
      if (millis() - initial > 100) {
        //        Serial.println(count);
        analogWrite(en1, (count + 1) * 10);
        analogWrite(en2, (count + 1) * 10);
        analogWrite(en3, (count + 1) * 10);
        analogWrite(en4, (count + 1) * 10);
        count++;
        initial = millis();
      }
    } else {
      if (Speed < 50) {
        analogWrite(en1, Speed);
        analogWrite(en2, Speed);
        analogWrite(en3, Speed);
        analogWrite(en4, Speed);
      }
      count = 0;
      is_end = true;
    }
  }
}

void Motor_directly(int Speed) {
  analogWrite(en1, Speed);
  analogWrite(en2, Speed);
  analogWrite(en3, Speed);
  analogWrite(en4, Speed);
}

void Motor_brakes(int Speed) {
  if (count < (Speed / 10)) {
    if (millis() - initial > 100) {
      analogWrite(en1, Speed - (count + 1) * 10);
      analogWrite(en2, Speed - (count + 1) * 10);
      analogWrite(en3, Speed - (count + 1) * 10);
      analogWrite(en4, Speed - (count + 1) * 10);
      count++;
      initial = millis();
    }
  } else {
    count = 0;
    Motor_reset();
    mode_code = -1;
  }
}

void Motor_brakes_with_time(int Speed, int after_time) {

  if (millis() - initial > after_time && !is_brake) {
    is_brake = true;
  }
  if (is_brake) {
    if (count < (Speed / 10)) {
      if (millis() - initial > 100) {
        analogWrite(en1, Speed - (count + 1) * 10);
        analogWrite(en2, Speed - (count + 1) * 10);
        analogWrite(en3, Speed - (count + 1) * 10);
        analogWrite(en4, Speed - (count + 1) * 10);
        count++;
        initial = millis();
      }
    } else {
      count = 0;
      is_brake = false;
      Motor_reset();
      mode_code = -1;
    }
  }
}

void Motor_brakes_with_sonar(int Speed, int min_speed) {
  if (count < ((Speed - min_speed) / 10)) {
    if (millis() - initial > 100) {
      Serial.print("brakes step=");
      Serial.print(count);
      Serial.print(", speed=");
      Serial.println(Speed - (count + 1) * 10);
      analogWrite(en1, Speed - (count + 1) * 10);
      analogWrite(en2, Speed - (count + 1) * 10);
      analogWrite(en3, Speed - (count + 1) * 10);
      analogWrite(en4, Speed - (count + 1) * 10);
      count++;
      initial = millis();
    }
  }
}

void Motor_full_work(int Speed, int Time) {
  if (is_start == true) {
    initial = millis();
    is_start = false;
  } else if (is_start == false && millis() - initial < Time) {
    analogWrite(en1, Speed);
    analogWrite(en2, Speed);
    analogWrite(en3, Speed);
    analogWrite(en4, Speed);
  } else {
    is_start = true;
  }
}

void m_type_correction_angle() {
  safety_around_angle(-relative_yaw);
}

void safety_around_angle(int angle) {
  if (!is_started) {
#if Pattern == 'A'
    sonic_servoL.write(0);
    sonic_servoR.write(0);
#endif
    is_started = true;
    is_end = false;
    base_yaw = relative_yaw;
    if (angle > 0) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      digitalWrite(in5, LOW);
      digitalWrite(in6, HIGH);
      digitalWrite(in7, HIGH);
      digitalWrite(in8, LOW);
    } else {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      digitalWrite(in5, HIGH);
      digitalWrite(in6, LOW);
      digitalWrite(in7, LOW);
      digitalWrite(in8, HIGH);
    }
    goal_yaw = base_yaw + angle;
    if (goal_yaw > 180) {
      goal_yaw -= 360;
    } else if (goal_yaw < -180) {
      goal_yaw += 360;
    }
    analogWrite(en1, 40);
    analogWrite(en2, 40);
    analogWrite(en3, 40);
    analogWrite(en4, 40);
  } else {
    //    Motor_start(50);
    Serial.print("target angle: ");
    Serial.print(goal_yaw);
    Serial.print(" now: ");
    Serial.println(relative_yaw);

    if (abs(relative_yaw - goal_yaw) < 0.5) {
      Serial.println("到達目標角度!!!");
      Motor_reset();
      //      is_started=false;
      is_started = false;
      Motor_reset();
#if Pattern == 'A'
      is_correction_angle = true;
#endif
    }
  }
  //  Motor_start(Speed);
  //  Motor_brakes(Speed);
}

void m_type_Leftward(int Speed) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
  Motor_start(Speed);
  //  Motor_brakes_with_time(Speed, Time);
}

void m_type_Rightward(int Speed) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);
  Motor_start(Speed);
  //  Motor_brakes_with_time(Speed, Time);
}

void m_type_Forward(int Speed) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
  Motor_start(Speed);
  //  Motor_brakes_with_time(Speed, Time);
}

void m_type_Backward(int Speed) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);
  Motor_start(Speed);
  //  Motor_brakes_with_time(Speed, Time);
}

void m_type_LeftBackward(int Speed) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  digitalWrite(in7, LOW);
  digitalWrite(in8, LOW);
  Motor_start(Speed);
  //  Motor_brakes_with_time(Speed, Time);
}

void m_type_LeftForward(int Speed) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  digitalWrite(in7, LOW);
  digitalWrite(in8, LOW);
  Motor_start(Speed);
  //  Motor_brakes_with_time(Speed, Time);
}

void m_type_RightBackward(int Speed) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  digitalWrite(in5, LOW);
  digitalWrite(in6, LOW);
  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);
  Motor_start(Speed);
  //  Motor_brakes_with_time(Speed, Time);
}

void m_type_RightForward(int Speed) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  digitalWrite(in5, LOW);
  digitalWrite(in6, LOW);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
  Motor_start(Speed);
}

void m_type_Leftward(int Speed, int Time) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
  Motor_start(Speed);
  Motor_brakes_with_time(Speed, Time);
}

void m_type_Rightward(int Speed, int Time) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);
  Motor_start(Speed);
  Motor_brakes_with_time(Speed, Time);
}

void m_type_Forward(int Speed, int Time) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
  Motor_start(Speed);
  Motor_brakes_with_time(Speed, Time);
}

void m_type_Backward(int Speed, int Time) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);
  Motor_start(Speed);
  Motor_brakes_with_time(Speed, Time);
}

void m_type_RightAround(int Speed, int Time) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);
  Motor_start(Speed);
  Motor_brakes_with_time(Speed, Time);
}

void m_type_LeftAround(int Speed, int Time) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
  Motor_start(Speed);
  Motor_brakes_with_time(Speed, Time);
}

void m_type_LeftBackward(int Speed, int Time) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  digitalWrite(in7, LOW);
  digitalWrite(in8, LOW);
  Motor_start(Speed);
  Motor_brakes_with_time(Speed, Time);
}

void m_type_LeftForward(int Speed, int Time) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  digitalWrite(in7, LOW);
  digitalWrite(in8, LOW);
  Motor_start(Speed);
  Motor_brakes_with_time(Speed, Time);
}

void m_type_RightBackward(int Speed, int Time) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  digitalWrite(in5, LOW);
  digitalWrite(in6, LOW);
  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);
  Motor_start(Speed);
  Motor_brakes_with_time(Speed, Time);
}

void m_type_RightForward(int Speed, int Time) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  digitalWrite(in5, LOW);
  digitalWrite(in6, LOW);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
  Motor_start(Speed);
  Motor_brakes_with_time(Speed, Time);
}

void Motor1_test(int Speed) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(en1, Speed);
}

void Motor2_test(int Speed) {
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(en2, Speed);
}

void Motor3_test(int Speed) {
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  analogWrite(en3, Speed);
}

void Motor4_test(int Speed) {
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
  analogWrite(en4, Speed);
}

void move_step(int goal_x, int goal_y, int go_STEP) {
  //  Serial.println("move_step_test");
  switch (step_move) {
    case 0:
      if (goal_x > 0) {
        if (distance_x > goal_x) {
          step_move = 10;
        } else {
          step_move = 20;
        }
      } else {
        step_move = 60;
      }
      break;
    case 10:
      if (distance_x - goal_x > 0.5) {
        if (distance_x - goal_x > 20) {
          m_type_Leftward(40);
          Serial.println("向左走");
        } else {
          Motor_brakes_with_sonar(40, 20);
          Serial.print("向左減速x距離=");
          Serial.println(distance_x - goal_x);
        }
      } else {
        count = 0;
        stop_x = millis();
        step_move = 51;
      }
      break;
    case 20:
      if (distance_x - goal_x < -0.5) {
        if (distance_x - goal_x < -20) {
          m_type_Rightward(40);
          Serial.println("向右走");
        } else {
          Motor_brakes_with_sonar(40, 30);
          Serial.print("向右減速x距離=");
          Serial.println(distance_x - goal_x);
        }
      } else {
        count = 0;
        stop_x = millis();
        step_move = 51;
      }
      break;
    case 51:
      if (millis() - stop_x < 200) {
        Serial.println("到達x點等待200ms");
        Motor_reset();
      } else {
        step_move = 60;
      }
      break;
    case 60:
      if (goal_y > 0) {
        if (distance_y > goal_y) {
          step_move = 30;
        } else {
          step_move = 40;
        }
      } else {
        STEP = go_STEP;
        step_move = 0;
      }
      break;
    case 30:
      if (distance_y - goal_y > 0.5) {
        if (distance_y - goal_y > 20) {
          m_type_Forward(70);
          Serial.println("向前走");
        } else {
          Motor_brakes_with_sonar(70, 60);
          Serial.print("向前減速y距離=");
          Serial.println(distance_y - goal_y);
        }
      } else {
        count = 0;
        stop_y = millis();
        step_move = 52;
      }
      break;
    case 40:
      if (distance_y - goal_y < -0.5) {
        if (distance_y - goal_y < -20) {
          m_type_Backward(70);
          Serial.println("向後走");
        } else {
          Motor_brakes_with_sonar(70, 60);
          Serial.print("向後減速y距離=");
          Serial.println(distance_y - goal_y);
        }
      } else {
        count = 0;
        stop_y = millis();
        step_move = 52;
      }
      break;
    case 52:
      if (millis() - stop_y < 200) {
        Serial.println("到達y點等待200ms");
        Motor_reset();
      } else {
        STEP = go_STEP;
        step_move = 0;
      }
      break;
  }
}

void move_step(int goal_x, int goal_y) {
  //  Serial.println("move_step_test");
  switch (step_move) {
    case 0:
      if (goal_x > 0) {
        if (distance_x > goal_x) {
          step_move = 10;
        } else {
          step_move = 20;
        }
      } else {
        step_move = 60;
      }
      break;
    case 10:
      if (distance_x - goal_x > 0.5) {
        if (distance_x - goal_x > 20) {
          m_type_Leftward(40);
          Serial.println("向左走");
        } else {
          Motor_brakes_with_sonar(40, 30);
          Serial.print("向左減速x距離=");
          Serial.println(distance_x - goal_x);
        }
      } else {
        stop_x = millis();
        step_move = 51;
      }
      break;
    case 20:
      if (distance_x - goal_x < -0.5) {
        if (distance_x - goal_x < -20) {
          m_type_Rightward(40);
          Serial.println("向右走");
        } else {
          Motor_brakes_with_sonar(40, 30);
          Serial.print("向右減速x距離=");
          Serial.println(distance_x - goal_x);
        }
      } else {
        stop_x = millis();
        step_move = 51;
      }
      break;
    case 51:
      if (millis() - stop_x < 200) {
        Serial.println("到達x點等待200ms");
        Motor_reset();
      } else {
        step_move = 60;
      }
      break;
    case 60:
      if (goal_y > 0) {
        if (distance_y > goal_y) {
          step_move = 30;
        } else {
          step_move = 40;
        }
      } else {
        STEP++;
        step_move = 0;
      }
      break;
    case 30:
      if (distance_y - goal_y > 0.5) {
        if (distance_y - goal_y > 20) {
          m_type_Forward(80);
          Serial.println("向前走");
        } else {
          Motor_brakes_with_sonar(80, 60);
          Serial.print("向前減速y距離=");
          Serial.println(distance_y - goal_y);
        }
      } else {
        stop_y = millis();
        step_move = 52;
      }
      break;
    case 40:
      if (distance_y - goal_y < -0.5) {
        if (distance_y - goal_y < -20) {
          m_type_Backward(80);
          Serial.println("向後走");
        } else {
          Motor_brakes_with_sonar(80, 60);
          Serial.print("向後減速y距離=");
          Serial.println(distance_y - goal_y);
        }
      } else {
        stop_y = millis();
        step_move = 52;
      }
      break;
    case 52:
      if (millis() - stop_y < 200) {
        Serial.println("到達y點等待200ms");
        Motor_reset();
      } else {
        STEP++;
        step_move = 0;
      }
      break;
  }
}

void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = (sonar[currentSensor].ping_result / 2) / 29.1;
}

void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
  // The following code would be replaced with your code that does something with the ping results.
  distance_x = cm[0];
  distance_L = cm[0];
  //  Serial.print("distance_x = ");
  //  Serial.print(distance_x);
  //  Serial.print("cm ");
  distance_y = cm[1];
  distance_R = cm[1];
  //  Serial.print("distance_y = ");
  //  Serial.print(distance_y);
  //  Serial.println("cm ");
}

void sonar_update() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
}

void turn_update() {
  IR_turn_val = digitalRead(IR_turns_sensor);
  if (IR_turn_val > pre_IR_turn_val) {
    turn_total++;
    //    Serial.println(turn_count);
  }
  pre_IR_turn_val = IR_turn_val;
}

void setup() {
  pinMode(IR_turns_sensor, INPUT);
  pinMode(start_bt, INPUT);
  pinMode(Buzzer, OUTPUT);
  digitalWrite(Buzzer, HIGH);
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

#if Pattern == 'A'
  sonic_servoR.attach(Rsonic_servo);
  sonic_servoL.attach(Lsonic_servo);
#endif

  Serial.begin(115200);
#if Pattern == 'R'  
  irrecv.blink13(true); // 設為true的話，當收到訊號時，腳位13的LED便會閃爍
  irrecv.enableIRIn(); // 啟動接收
#endif
  Serial.println("start");
  Motor_reset();

  mpu6050_setup();

  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
}

void loop() {

  if (!gyro_ready) {
    return;
  }

  sonar_update();
  mpu6050_update();
  turn_update();

  original_start = digitalRead(start_bt);
  if (Start != original_start) {
    if (original_start) {
      Serial.println("自動模式");
      step_start = millis();
      loop_x = d5;
      loop_y = d2 - (2 * car_y);
      step_move = 0;
      STEP = 1;//測試先用-1
    } else {
      Serial.println("手動模式");
      mode_code = 0;
    }
    Start = original_start;
  }

  if (!Start) {
#if Pattern == 'R'
    IR_update();
    switch (mode_code)
    {
      case 0:
        Motor_reset();
        //        safety_around_angle(90);
        //        m_type_Forward(100, 0);
        break;
      case 16738455://0
        Motor_reset();
        //              Serial.println("000");
        break;
      case 16724175://1
        m_type_Forward(70, 1000);
        break;
      case 16718055://2
        m_type_Backward(70, 1000);
        //        safety_around_angle(90);
        //      Serial.println("222");
        break;
      case 16743045://3
        m_type_Rightward(40, 1000);
        //        safety_around_angle(-90);
        //      Serial.println("333");
        break;
      case 16716015://4
        m_type_Leftward(40, 1000);
        //        safety_around_angle(45);
        break;
      case 16726215://5
        //        m_type_RightAround(50, 0);
        safety_around_angle(-45);
        break;
      case 16734885://6
        //          safety_around_angle(45);
        m_type_correction_angle();
        Serial.println("666");
        break;
      case 16728765://7
        m_type_RightForward(100, 1000);
        Serial.println(yaw);
        break;
      case 16730805://8
        m_type_LeftBackward(100, 1000);
        Serial.println("888");
        break;
      case 16732845://9
        Motor_brakes(50);
        break;
      default:
        //        Serial.println("default");
        Motor_reset();
        is_end = false;
        break;

    }
#endif
  } else {
#if Pattern == 'A'
    switch (STEP) {
      case -2:
        move_step(50, 120);
        //          Serial.println(motor_start);
        break;
      case -1:
        Serial.println("case -1");
        break;
      case 1://起步到顏色看板
        move_step(d1, d2);
        break;
      case 2://對者顏色看板前進
        //          move_step(d3, d2-car_y);
        move_step(d3, -1);
        break;
      case 3://向左走到牆

        //          move_step(loop_x, loop_y);
        break;
      case 4://掃球
        if (sweep_ball_step > 3) {
          sweep_ball_step = 0;
        }
        switch (sweep_ball_step) {
          case 0:
            Move('F', 5, 60, 100);  //方向(前F後B左L右R),輪胎圈數,超音波距離,輪胎速度
            break;
          case 1:
            Move('R', 20, 50, 100);
            break;
          case 2:
            Move('F', 5, 60, 100);
            break;
          case 3:
            Move('L', 5, 60, 100);
            break;
        }

        //        if (distance_y > stop_collect_y) {
        //          if (loop_x == d5) {
        //            loop_x = d3;
        //          } else {
        //            loop_x = d5;
        //          }
        //          loop_y -= car_y;
        //          STEP--;
        //        } else {
        //          STEP++;
        //        }
        break;
      case 5:
        //        Motor_brakes();
        break;
    }
#endif
  }

  //    Serial.print(" now: ");
  //    Serial.println(relative_yaw);
  //    Serial.print("相對角度: ");
  //    Serial.println(relative_yaw);

}
