#include <MPU6050_6Axis_MotionApps20.h>


MPU6050 mpu;
//----------gobal_var---------

unsigned long step_start;
unsigned long initial;
float original_z = 0;
bool gyro_ready = false;
float relative_yaw;
float base_yaw;
float goal_yaw;
int team_color = 1;
bool run_step = false;
bool led_state = false;
bool plus_ball_shotted = false;
bool yaw_back_0 = false;
bool in_change_yaw = false;


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
const int open_game = 49;
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
const int move_A = 31;
const int move_B = 30;
const int move_slow = 29;
const int change_pid = 28;
//-----------------------------
const int yaw_in_0 = 11;
const int yellow_yaw = 24;
const int orange_yaw = 25;
const int over_plus_ball = 23;

//-----------------------------

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

//----------pid------------
int kp = 3;
int kd = 2;
int kp1 = 1.5;
int kd1 = 0.5;
int speed_n = 100;
int speed_ne = -110;
int speed_pu = 110;
int speed_n2 = 50;
int speed_ne2 = -60;
int speed_pu2 = 60;
int speed_n1 = 90;
int speed_ne1 = -100;
int speed_pu1 = 100;
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
long pidtest_time;
int lai = 0;


//---------func-----------
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
  e = relative_yaw;
  e1 = abs(relative_yaw);
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
  e = relative_yaw;
  e1 = abs(relative_yaw);
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
  analogWrite(en1, 160);
  analogWrite(en2, 160);
  analogWrite(en3, 160);
  analogWrite(en4, 160);
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
            if (led_state == false) {
              led_off();
              led_state = true;
            } else {
              led_red();
              led_state = false;
            }
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
    led_green();
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

  pinMode(move_A, INPUT_PULLUP);//停(high,high)左(low,high)
  pinMode(move_B, INPUT_PULLUP);//右(high,low)前(low,low)
  //  pinMode(is_shot_plus_pin, OUTPUT);
  //  digitalWrite(is_shot_plus_pin, HIGH);
  pinMode(is_shot_pin, OUTPUT);
  digitalWrite(is_shot_pin, HIGH);


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

  pinMode(open_game, OUTPUT);
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
  pinMode(move_slow, INPUT_PULLUP);
  pinMode(change_pid, INPUT_PULLUP);

  pinMode(Buzzer1, OUTPUT);
  pinMode(Buzzer2, OUTPUT);
  led_red();
  Wire.begin();
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("start");

  mpu6050_setup();
  Motor_reset();

  //  if (digitalRead(team_color_pin) == LOW) {
  //    team_color = 1;//黃
  //  } else {
  //    team_color = 2;//橘
  //  }




}

void loop() {
  if (run_step == false) {
    if (digitalRead(start_bt_pin) == HIGH) {
      digitalWrite(open_game, LOW);//HIGH 180 LOW 90
      run_step = true;
      //      digitalWrite(is_start_pin, LOW);
    }
  }

  if (!gyro_ready) {
    //    Serial.println("not ready");
    return;
  }
  //------sensor更新------

  //  ks103_update();
  //---------------------

  //=======debug=========
  mpu6050_update();
  Serial.print("relative_yaw:");
  Serial.println(relative_yaw);
  //=====================
  //move
  if(in_change_yaw == false){
  if (digitalRead(move_A) == HIGH and digitalRead(move_B) == HIGH) {
    Motor_reset();
  } else if (digitalRead(move_A) == LOW and digitalRead(move_B) == HIGH and digitalRead(move_slow) == HIGH) {
    PIDR1();
  } else if (digitalRead(move_A) == HIGH and digitalRead(move_B) == LOW and digitalRead(move_slow) == HIGH) {
    PIDL1();
  } else if (digitalRead(move_A) == LOW and digitalRead(move_B) == LOW and digitalRead(move_slow) == HIGH) {
    PIDF();
  } else if (digitalRead(move_A) == LOW and digitalRead(move_B) == HIGH and digitalRead(move_slow) == LOW) {
    PIDR2();
  } else if (digitalRead(move_A) == HIGH and digitalRead(move_B) == LOW and digitalRead(move_slow) == LOW) {
    PIDL2();
  } else if (digitalRead(move_A) == LOW and digitalRead(move_B) == LOW and digitalRead(move_slow) == LOW) {
    PIDF1();
  }
  }
  //change_pid
  if (digitalRead(change_pid) == LOW) {
    speed_n1 = 70;
    speed_ne1 = -80;
    speed_pu1 = 80;
  } else {
    speed_n1 = 90;
    speed_ne1 = -100;
    speed_pu1 = 100;
  }
  //shot plus ball
  if(plus_ball_shotted == false){
    if (digitalRead(yellow_yaw) == LOW){
      in_change_yaw = true;
      if(relative_yaw < 42){
        LeftAround1();
        pidtest_time = millis();
      }else if(millis()-pidtest_time < 14000 and digitalRead(over_plus_ball) == HIGH){
        Motor_reset();
      }else{
        plus_ball_shotted = true;
      }
  }
    if(yaw_back_0 == false){
     if(plus_ball_shotted == true){
       if(relative_yaw > 0.5){
         RightAround1();
         pidtest_time = millis();
       }else{
         digitalWrite(yaw_in_0,LOW);
         yaw_back_0 = true;
         in_change_yaw = false;
       }
     }
    }
}
