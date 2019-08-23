#include <MPU6050_6Axis_MotionApps20.h>
#include "Wire.h"

#define KS103_L 0x74
#define KS103_R 0x75


MPU6050 mpu;
unsigned long ks103_time;
int ks103_state = 0;
int distance_R, distance_L;
unsigned long step_start;
unsigned long initial;
float original_z = 0;
bool gyro_ready = false;
float relative_yaw;
float base_yaw;
float goal_yaw;


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
const int angle180 = 51;
const int angle135 = 53;
const int team_color_bt = 24;
const int start_bt = 23;
const int Buzzer = 38;
const int IR_turns_sensor = 37;
const int collect_ball_pin = 35;
const int pullup_ball_pin = 36;
const int shot_ball_pin = 34;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

//----------pid------------
int kp = 8;
int kd = 5;
int kp1 = 5;
int kd1 = 3;
int speed_n = 100;
int speed_ne = -110;
int speed_pu = 110;
//int speed_n2 = 50;
//int speed_ne2 = -60;
//int speed_pu2 = 60;
int speed_n1 = 120;
int speed_ne1 = -130;
int speed_pu1 = 130;
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
int flag = 1;
long pidtest_time;

void PIDR() {
  e = relative_yaw;
  e1 = abs(relative_yaw);
  control = kp * e1 + kd * (e1 - e_pre);
  speed_L = speed_n + control;
  // speed_R = speed_n - control;
  if (speed_L > speed_pu) {
    speed_L = speed_pu;
  }
  //  if (speed_L < speed_ne) {
  //    speed_L = speed_ne;
  //  }
  if (speed_R > speed_pu) {
    speed_R = speed_pu;
  }
  //  if (speed_R < speed_ne) {
  //    speed_R = speed_ne;
  //  }
  speed_L = abs(speed_L);
  //  speed_R = abs(speed_R);
  speed_LI = floor(speed_L);
  //  speed_RI = floor(speed_R);
  e_pre = e1;
  if (e > 0) {
    RightAround();
  }
  else if (e < 0) {
    LeftAround();
  }
  else {
    Rightward();
  }
}

void PIDL() {
  e = relative_yaw;
  e1 = abs(relative_yaw);
  control = kp * e1 + kd * (e1 - e_pre);
  speed_L = speed_n + control;
  //  speed_R = speed_n - control;
  if (speed_L > speed_pu) {
    speed_L = speed_pu;
  }
  //  if (speed_L < speed_ne) {
  //    speed_L = speed_ne;
  //  }
  if (speed_R > speed_pu) {
    speed_R = speed_pu;
  }
  //  if (speed_R < speed_ne) {
  //    speed_R = speed_ne;
  //  }
  speed_L = abs(speed_L);
  //  speed_R = abs(speed_R);
  speed_LI = floor(speed_L);
  //  speed_RI = floor(speed_R);
  e_pre = e1;
  if (e > 0) {
    RightAround();
  }
  else if (e < 0) {
    LeftAround();
  }
  else {
    Leftward();
  }
}

void PIDF() {
  e = relative_yaw;
  e1 = abs(relative_yaw);
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
  if (e > 0) {
    RightAround();
  }
  else if (e < 0) {
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

  //  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

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
  }
}

void servo_reset() {
  digitalWrite(angle90, LOW);
  digitalWrite(angle180, LOW);
  digitalWrite(angle135, LOW);
}

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

  pinMode(angle90, OUTPUT);
  pinMode(angle180, OUTPUT);
  pinMode(angle135, OUTPUT);

  pinMode(collect_ball_pin, OUTPUT);
  pinMode(pullup_ball_pin, OUTPUT);
  pinMode(shot_ball_pin, OUTPUT);
  pinMode(Buzzer, OUTPUT);
  digitalWrite(Buzzer, HIGH);
  Wire.begin();
  setting_ks103(KS103_L, 0x75);
  setting_ks103(KS103_R, 0x75);
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("start");

  mpu6050_setup();
  servo_reset();
  Motor_reset();
  digitalWrite(angle180, HIGH);
}
void loop() {
  if (!gyro_ready) {
    return;
  }
  //------sensor更新------
  mpu6050_update();
  ks103_update();
  //---------------------

  //=======debug=========
  Serial.print("relative_yaw:");
  Serial.println(relative_yaw);
  Serial.print("L:");
  Serial.print(distance_L);
  Serial.print("R:");
  Serial.println(distance_R);
  //=====================


  if (distance_R < 200 and flag == 1)
  {
    PIDR();
    pidtest_time = millis();
  } else if (flag == 1) {
    if (millis() - pidtest_time < 1000) {
      Motor_reset();
    } else {
      flag++;
      pidtest_time = millis();
      servo_reset();
      digitalWrite(angle90, HIGH);
    }
  }

  if (flag == 2) {
    if (millis() - pidtest_time < 6000) {
      PIDF();
    } else {
      Motor_reset();
      flag++;
      servo_reset();
      digitalWrite(angle180, HIGH);
    }
  }

  if (distance_L > 50 and flag == 3) {
    PIDR();
    pidtest_time = millis();
  } else if (flag == 3) {
    if (millis() - pidtest_time < 1000) {
      Motor_reset();
    } else {
      flag++;
      pidtest_time = millis();
      servo_reset();
      digitalWrite(angle90, HIGH);
    }
  }


  if (flag == 4) {
    if (millis() - pidtest_time < 2000) {
      PIDF();
    } else {
      Motor_reset();
      flag++;
      servo_reset();
      digitalWrite(angle180, HIGH);
    }
  }


  if (distance_R > 50 and flag == 5)
  {
    PIDL();
    pidtest_time = millis();
  } else if (flag == 5) {
    if (millis() - pidtest_time < 1000) {
      Motor_reset();
    } else {
      flag++;
      pidtest_time = millis();
      servo_reset();
      digitalWrite(angle90, HIGH);
    }
  }


  if (flag == 6) {
    if (millis() - pidtest_time < 2000) {
      PIDF();
    } else {
      Motor_reset();
      flag++;
      servo_reset();
      digitalWrite(angle180, HIGH);
    }
  }


  if (distance_L > 50 and flag == 7)
  {
    PIDR();
    pidtest_time = millis();
  } else if (flag == 7) {
    if (millis() - pidtest_time < 1000) {
      Motor_reset();
    } else {
      flag++;
      pidtest_time = millis();
      servo_reset();
      digitalWrite(angle90, HIGH);
    }
  }


  if (flag == 8) {
    if (millis() - pidtest_time < 2000) {
      PIDF();
    } else {
      Motor_reset();
      flag++;
      servo_reset();
      digitalWrite(angle180, HIGH);
    }
  }


  if (distance_R > 50 and flag == 9)
  {
    PIDL();
    pidtest_time = millis();
  } else if (flag == 9) {
    if (millis() - pidtest_time < 1000) {
      Motor_reset();
    } else {
      flag++;
      pidtest_time = millis();
      servo_reset();
      digitalWrite(angle90, HIGH);
    }
  }


  if (flag == 10) {
    if (millis() - pidtest_time < 2000) {
      PIDF();
    } else {
      Motor_reset();
      flag++;
      servo_reset();
      digitalWrite(angle180, HIGH);
    }
  }


  if (distance_L > 50 and flag == 11)
  {
    PIDR();
    pidtest_time = millis();
  } else if (flag == 11) {
    if (millis() - pidtest_time < 1000) {
      Motor_reset();
    } else {
      flag++;
      pidtest_time = millis();
      servo_reset();
      digitalWrite(angle90, HIGH);
    }
  }


  if (flag == 12) {
    if (millis() - pidtest_time < 2000) {
      PIDF();
    } else {
      Motor_reset();
      flag++;
      servo_reset();
      digitalWrite(angle180, HIGH);
    }
  }


  if (distance_R > 50 and flag == 13)
  {
    PIDL();
    pidtest_time = millis();
  } else if (flag == 13) {
    if (millis() - pidtest_time < 1000) {
      Motor_reset();
    } else {
      flag++;
      pidtest_time = millis();
      servo_reset();
      digitalWrite(angle90, HIGH);
    }
  }


  if (flag == 14) {
    if (millis() - pidtest_time < 2000) {
      PIDF();
    } else {
      Motor_reset();
      //   flag++;
      servo_reset();
      digitalWrite(angle180, HIGH);
    }
  }




}
