#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include "Wire.h"

#define KS103_L 0x74
#define KS103_R 0x75
#define Pattern 'A'//A,AUTO;R,ROMOTE#R要改矩陣鍵盤


//==========pin================
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
const int Lsonic_servo = 8;
const int Rsonic_servo = 9;
const int team_color_bt = 24;
const int start_bt = 23;
const int Buzzer = 38;
const int IR_turns_sensor = 37;

char team_color = 'Y';
bool team_color_bool = false;
bool Start = false;
bool original_start = false;
int STEP = 0;
int sweep_ball_step = 0;
int step_move = 0;
int find_color_borad_step = 0;

int kp=8;
int kd=5;
int speed_n=50;
int speed_ne=-60;
int speed_pu=60;
int e;
int e_pre=0;
int control;
int speed_L;
int speed_R;
int speed_LI;
int speed_RI;


bool set_turn = false;
int turn_total = 0;
int turn_count = 0;
int brake_Speed = 0;
int pre_IR_turn_val = 1;
int IR_turn_val;
bool is_mode;
int mode_code = 0;

bool is_Move_start = false;
bool is_correction_angle = false;

unsigned long ks103_time;
int ks103_state = 0;
int distance_x, distance_y;
int distance_R, distance_L;

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
        Serial.println("F");
        sonic_servoL.write(15);
        sonic_servoR.write(180);
        break;
      case 'B'://B
        Serial.println("B");
        sonic_servoL.write(15);
        sonic_servoR.write(180);
        break;
      case 'L'://L
        Serial.println("L");
        sonic_servoL.write(105);
        sonic_servoR.write(90);
        break;
      case 'R'://R
        Serial.println("R");
        sonic_servoL.write(105);
        sonic_servoR.write(90);
        break;
      case '3'://LF
        sonic_servoL.write(60);
        sonic_servoR.write(135);
        break;
      case '1'://RF
        sonic_servoL.write(60);
        sonic_servoR.write(135);
        break;
      case '4'://LB
        sonic_servoL.write(60);
        sonic_servoR.write(135);
        break;
      case '2'://RB
        sonic_servoL.write(60);
        sonic_servoR.write(135);
        break;
    }
  } else {
    //    Serial.println(turn_total - turn_count);
    Serial.print("R:");
    Serial.print(distance_R);
    Serial.print("L:");
    Serial.println(distance_L);
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
        Serial.println("slow_down");
        if ((Speed - (turn_total - turn_count) * 2) < 50) {//移動最低速
          Motor_directly(50);
        } else {
          Motor_directly(Speed - (turn_total - turn_count) * 2);
        }
      } else {
        switch (Direction) {
          case 'F'://F
            if (is_correction_angle == true) {
              Serial.print("distanc:");
              Serial.println(distance_L);
              if (int(distance_L) <= 60 and int(distance_L) != 0) { //進入發球距離
                STEP++;
              } else {
                is_Move_start = false;
                sweep_ball_step++;
              }//
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
            if (int(distance_L) <= sonic_Distance and int(distance_L) != 0) {
              if (is_correction_angle == true) {
                is_Move_start = false;
                sweep_ball_step++;
              } else {
                m_type_correction_angle();
              }
            }
            break;
          case 'R'://R
            if (int(distance_R) <= sonic_Distance and int(distance_R) != 0) {
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
            break;
        }
      }
    }
  }
}

void First_Move(char Direction, int Turns, int sonic_Distance, int Speed) { //方向(前F後B左L右R),輪胎圈數,超音波距離,輪胎速度
  if (is_Move_start == false) {                                   //向(方向)用(輪胎速度)轉(輪胎圈數)後用低速調整至(超音波距離)
    is_Move_start = true;
    is_correction_angle = false;
    turn_count = turn_total;
    switch (Direction) {
      case 'L'://L
        Serial.println("L");
        sonic_servoL.write(105);
        sonic_servoR.write(90);
        break;
      case 'R'://R
        Serial.println("R");
        sonic_servoL.write(105);
        sonic_servoR.write(90);
        break;
    }
  } else {
    //    Serial.println(turn_total - turn_count);
    Serial.print("R:");
    Serial.print(distance_R);
    Serial.print("L:");
    Serial.println(distance_L);
    if (turn_total - turn_count < Turns) {
      switch (Direction) {
        case 'L'://L
          m_type_Leftward(Speed);
          break;
        case 'R'://R
          m_type_Rightward(Speed);
          break;
      }
    } else {
      if (((turn_total - turn_count) - Turns) < 5) {
        Serial.println("slow_down");
        if ((Speed - (turn_total - turn_count) * 2) < 50) {//移動最低速
          Motor_directly(50);
        } else {
          Motor_directly(Speed - (turn_total - turn_count) * 2);
        }
      } else {
        switch (Direction) {
          case 'L'://L
            if (int(distance_R) <= sonic_Distance and int(distance_R) != 0) {
              if (is_correction_angle == true) {
                is_Move_start = false;
                STEP++;
              } else {
                m_type_correction_angle();
              }
            }
            break;
          case 'R'://R
            if (int(distance_L) <= sonic_Distance and int(distance_L) != 0) {
              if (is_correction_angle == true) {
                is_Move_start = false;
                STEP++;
              } else {
                m_type_correction_angle();
              }
            }
            break;
        }
      }
    }
  }
}

#endif


MPU6050 mpu;

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
  Wire.begin();
  float first_yaw = 0;
  int counter_ready = 0;
  float last_yaw = 999;

  // join I2C bus (I2Cdev library doesn't do this automatically)

//    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

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

void Motor_brakes_with_turn(int Speed, int Turn, int min_Speed, int brakes_turn) {
  if (set_turn == false) {
    turn_count = turn_total;
    is_correction_angle = false;
  } else if ((turn_total - turn_count) > Turn && !is_brake) {
    is_brake = true;
  }
  if (is_brake) {
    if (((turn_total - turn_count) - Turn) < brakes_turn) {
      brake_Speed = Speed - (((Speed - min_Speed) / brakes_turn) * (count + 1));
      analogWrite(en1, brake_Speed);
      analogWrite(en2, brake_Speed);
      analogWrite(en3, brake_Speed);
      analogWrite(en4, brake_Speed);
      count++;
    } else if (is_correction_angle == false) {
      m_type_correction_angle();
    } else {
      count = 0;
      brake_Speed = 0;
      is_brake = false;
      set_turn = false;
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
    sonic_servoL.write(105);
    sonic_servoR.write(90);
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
void PID() {
   e=relative_yaw;
   control=kp*e+kd*(e-e_pre);
   speed_L=speed_n+control;
   speed_R=speed_n-control;
   if (speed_L>speed_pu) {
    speed_L=speed_pu;
   }
   if (speed_L<speed_ne) {
    speed_L=speed_ne;
   }
   if (speed_R>speed_pu) {
    speed_R=speed_pu;
   }
   if (speed_R<speed_ne) {
    speed_R=speed_ne;
   }  
   speed_L=abs(speed_L);
   speed_R=abs(speed_R);
   speed_LI=floor(speed_L);
   speed_RI=floor(speed_R);
   e_pre=e;
   if (e>0) {
    RightAround();
   }
   else if (e<0) {
   LeftAround();
   }
   else {
    Leftward();
  //  Rightward();
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
  analogWrite(en1, speed_RI);
  analogWrite(en3, speed_RI);
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
  analogWrite(en2, speed_RI);
  analogWrite(en4, speed_RI);
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

void turn_update() {
  IR_turn_val = digitalRead(IR_turns_sensor);
  if (IR_turn_val > pre_IR_turn_val) {
    turn_total++;
    //    Serial.println(turn_count);
  }
  pre_IR_turn_val = IR_turn_val;
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
    ks103_time = millis();
    ks103_state++;
  } else if ((millis() - ks103_time) > 1 and ks103_state == 1) {
    Wire.beginTransmission(KS103_L);
    Wire.write(byte(0x02));
    Wire.endTransmission();
    Wire.requestFrom(KS103_L, 2);
    if (2 <= Wire.available()) {
      distance_L = Wire.read();
      distance_L =  distance_L << 8;
      distance_L |= Wire.read();
      distance_L = distance_L / 10;
      ks103_time = millis();
      ks103_state++;
    }
  } else if ((millis() - ks103_time) > 100 and ks103_state == 2) {
    Wire.beginTransmission(KS103_R);
    Wire.write(byte(0x02));
    Wire.write(0xb4);     //量程设置为5m 带温度补偿
    Wire.endTransmission();
    ks103_time = millis();
    ks103_state++;
  } else if ((millis() - ks103_time) > 1 and ks103_state == 3) {
    Wire.beginTransmission(KS103_R);
    Wire.write(byte(0x02));
    Wire.endTransmission();
    Wire.requestFrom(KS103_R, 2);
    if (2 <= Wire.available()) {
      distance_R = Wire.read();
      distance_R =  distance_R << 8;
      distance_R |= Wire.read();
      distance_R = distance_R / 10;
      ks103_state++;
      ks103_time = millis();
    }
  } else if ((millis() - ks103_time) > 100 and ks103_state == 4) {
    ks103_state = 0;
  }
}

void setup() {

  Wire.begin();
  setting_ks103(KS103_L, 0x75);
  setting_ks103(KS103_R, 0x75);
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

//#if Pattern == 'A'
//  sonic_servoR.attach(Rsonic_servo);
//  sonic_servoL.attach(Lsonic_servo);
//#endif

  Serial.begin(115200);
  Serial.println("start");
  Motor_reset();

  mpu6050_setup();

}

void loop() {
   
  if (!gyro_ready) {
    return;
  }

  mpu6050_update();

//  PID();
  turn_update();
  ks103_update();
  Serial.print("relative_yaw:");
  Serial.println(relative_yaw);
  Serial.print("L:");
  Serial.print(distance_L);
  Serial.print("R:");
  Serial.println(distance_R);

//  Start = digitalRead(start_bt);

//if(Start == True){
//#if Pattern == 'A'
//    switch (STEP) {
//      case -2:
//        move_step(50, 120);
//        //          Serial.println(motor_start);
//        break;
//      case -1:
//        Serial.println("case -1");
//        break;
//      case 1://起步到顏色看板
//        switch (find_color_borad_step) {
//          case 0:
//            sonic_servoL.write(105);
//            sonic_servoR.write(90);
//            find_color_borad_step++;
//            break;
//          case 1://固定行走圈數結束後判斷超音波
//            switch (team_color) {
//              case 'Y':
//                First_Move('R', 5, 200, 100);//方向(前F後B左L右R),輪胎圈數,超音波距離,輪胎速度
//              case 'O':
//                First_Move('L', 5, 200, 100);
//                break;
//            }
//            break;
//        }
//        break;
//      case 2://對者顏色看板前進
//        m_type_Forward(70);
//        Motor_brakes_with_turn(70, 10, 50, 5);
//        if (is_correction_angle == true) {
//          STEP++;
//        }
//        break;
//      case 3://向左走到牆//固定行走圈數結束後判斷超音波
//        switch (team_color) {
//          case 'Y':
//            Move('R', 5, 20, 70);//方向(前F後B左L右R),輪胎圈數,超音波距離,輪胎速度
//          case 'O':
//            Move('L', 5, 20, 70);
//            break;
//        }
//        if (is_correction_angle == true) {
//          STEP++;
//          sweep_ball_step = 0;
//        }
//        break;
//      case 4://掃球
//        //        Serial.print("Sweep_ball:");
//        //        Serial.println(sweep_ball_step);
//        if (sweep_ball_step > 3) {
//          sweep_ball_step = 0;
//        }
//        switch (sweep_ball_step) {
//          case 0:
//            Move('F', 5, 60, 100);  //方向(前F後B左L右R),輪胎圈數,超音波距離,輪胎速度
//            break;
//          case 1:
//            Move('R', 20, 50, 100);
//            break;
//          case 2:
//            Move('F', 5, 60, 100);
//            break;
//          case 3:
//            Move('L', 5, 60, 100);
//            break;
//        }
//        break;
//      case 5:
//        Serial.println("Shot");
//        //        Motor_brakes();
//        break;
//    }
//#endif
//}
}
