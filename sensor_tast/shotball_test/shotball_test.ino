/**
 * @file shotball_test.ino
 * @brief Logic test sketch for robot movement and shooting.
 *
 * This sketch tests the integration of MPU6050-based PID movement (Omnidirectional)
 * and KS103 distance sensors for a specific sequence of actions (moving to a position and shooting).
 * It includes PID control logic for forward, backward, left, and right movements while maintaining heading.
 */

#include <MPU6050_6Axis_MotionApps20.h>
#include "Wire.h"

#define KS103_L 0x74
#define KS103_R 0x75

MPU6050 mpu;
unsigned long ks103_time;//
int ks103_state = 0;
int distance_R, distance_L;
unsigned long step_start;
unsigned long initial;
float original_z = 0;
bool gyro_ready = false;
float relative_yaw;
float base_yaw;
float goal_yaw;
int team_color = 1;
bool run_step = false;

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
const int Buzzer1 = 38;
const int Buzzer2 = 36;
const int pixy_color_flag_pin = 37;
const int is_shot_pin = 27;
const int team_color_bt_pin = 34;
const int start_bt_pin = 35;
const int riseball_pin = 33;
const int sweepball_pin = 32;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

//----------pid------------
int kp = 3;
int kd = 2;
int kp1 = 3;
int kd1 = 2;
int speed_n = 100;
int speed_ne = -110;
int speed_pu = 110;
//int speed_n2 = 50;
//int speed_ne2 = -60;
//int speed_pu2 = 60;
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
int flag = 0;
long pidtest_time;
int lai = 0;

/**
 * @brief PID control for rightward movement.
 *
 * Adjusts motor speeds based on heading error to maintain straight line motion to the right.
 */
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

/**
 * @brief PID control for rightward movement (variant 1).
 *
 * Tighter threshold for correction (1 degree).
 */
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

/**
 * @brief PID control for leftward movement.
 *
 * Adjusts motor speeds based on heading error to maintain straight line motion to the left.
 */
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

/**
 * @brief PID control for leftward movement (variant 1).
 *
 * Tighter threshold for correction (1 degree).
 */
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

/**
 * @brief PID control for forward movement.
 *
 * Adjusts motor speeds based on heading error to maintain straight line motion forward.
 */
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

/**
 * @brief PID control for forward movement (variant 1).
 *
 * Tighter threshold for correction (1 degree).
 */
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


/**
 * @brief Rotates robot right using calculated speed.
 */
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

/**
 * @brief Rotates robot left using calculated speed.
 */
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

/**
 * @brief Rotates/moves forward-around.
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
 * @brief Rotates/moves backward-around.
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
 * @brief Moves robot rightward using base speed.
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
  analogWrite(en1, speed_n);
  analogWrite(en2, speed_n);
  analogWrite(en3, speed_n);
  analogWrite(en4, speed_n);
}

/**
 * @brief Rotates robot right using fixed speed 80.
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
  analogWrite(en1, 80);
  analogWrite(en3, 80);
  analogWrite(en2, 80);
  analogWrite(en4, 80);
}

/**
 * @brief Rotates robot left using fixed speed 80.
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
  analogWrite(en1, 80);
  analogWrite(en3, 80);
  analogWrite(en2, 80);
  analogWrite(en4, 80);
}

/**
 * @brief Moves robot leftward using base speed.
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
  analogWrite(en1, speed_n);
  analogWrite(en2, speed_n);
  analogWrite(en3, speed_n);
  analogWrite(en4, speed_n);
}

/**
 * @brief Moves robot leftward using fixed speed 60.
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
 * @brief Moves robot forward using fixed speed 160.
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
  analogWrite(en1, 160);
  analogWrite(en2, 160);
  analogWrite(en3, 160);
  analogWrite(en4, 160);
}

/**
 * @brief Moves robot forward using base speed `speed_n1`.
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
  analogWrite(en1, speed_n1);
  analogWrite(en2, speed_n1);
  analogWrite(en3, speed_n1);
  analogWrite(en4, speed_n1);
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

/**
 * @brief Interrupt Service Routine for MPU6050.
 */
void dmpDataReady() {
  mpuInterrupt = true;
}

/**
 * @brief Initializes MPU6050 with DMP.
 */
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
    led_green();
  } else {
    // if programming failed, don't try to do anything
    Serial.println("陀螺儀初始化錯誤!!!");
  }
}

/**
 * @brief Updates MPU6050 data and relative yaw.
 */
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

/**
 * @brief Checks if MPU data is ready (non-blocking).
 */
bool mpu6050_getyaw() {
  if (!mpuInterrupt && fifoCount < packetSize) {
    return true;
  } else {
    mpu6050_update();
    return false;
  }
}

/**
 * @brief Configures a KS103 sensor via I2C.
 */
void setting_ks103(byte addr, byte command) {
  Wire.beginTransmission(addr);
  Wire.write(byte(0x02));
  Wire.write(command);   // 发送降噪指令
  Wire.endTransmission();
  delay(1000);
}

/**
 * @brief Updates distance readings from KS103 sensors (non-blocking).
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
  }
}

/**
 * @brief Resets servo control pins to HIGH.
 */
void servo_reset() {
  digitalWrite(angle90, HIGH);
  digitalWrite(angle180, HIGH);
  digitalWrite(angle135, HIGH);
}

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
 * @brief Setup function.
 *
 * Initializes hardware, sensors, and state.
 */
void setup() {
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

  pinMode(angle90, OUTPUT);
  pinMode(angle180, OUTPUT);
  pinMode(angle135, OUTPUT);

  pinMode(riseball_pin, OUTPUT);
  pinMode(sweepball_pin, OUTPUT);
  digitalWrite(riseball_pin, LOW);
  digitalWrite(sweepball_pin, LOW);

  pinMode(pixy_color_flag_pin, OUTPUT);
  digitalWrite(pixy_color_flag_pin, HIGH);

  pinMode(team_color_bt_pin, INPUT_PULLUP);
  pinMode(start_bt_pin, INPUT_PULLUP);

  pinMode(Buzzer1, OUTPUT);
  pinMode(Buzzer2, OUTPUT);
  led_red();
  Wire.begin();
  setting_ks103(KS103_L, 0x75);
  setting_ks103(KS103_R, 0x75);
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("start");

  mpu6050_setup();
  servo_reset();
  Motor_reset();
  digitalWrite(angle180, LOW);
  if (digitalRead(team_color_bt_pin) == LOW) {
    team_color = 1;//黃
  } else {
    team_color = 2;//橘
  }

  digitalWrite(riseball_pin, HIGH);
  digitalWrite(sweepball_pin, HIGH);

}

/**
 * @brief Main loop.
 *
 * Executes a sequence of movements (using flag 29, 30...) when triggered by start button.
 * Uses PID to control movement direction and distance sensors to trigger transitions.
 */
void loop() {
  if (run_step == false) {
    if (digitalRead(start_bt_pin) == HIGH) {
      flag = 29;
      digitalWrite(riseball_pin, HIGH);
      digitalWrite(sweepball_pin, HIGH);
      run_step = true;
    }
  }
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

  if (distance_R < 275 and flag == 29 and lai == 0)
  {
    PIDR1();
    pidtest_time = millis();
  } else if (flag == 29) {
    if (millis() - pidtest_time < 1000) {
      Motor_reset();
      lai = 1;
    } else {
      flag++;
      pidtest_time = millis();
      lai = 0;
      servo_reset();
      digitalWrite(angle90, LOW);
      speed_n1 = 70;
      speed_ne1 = -80;
      speed_pu1 = 80;
      digitalWrite(is_shot_pin, LOW);
    }
  }

  if (distance_R > 71 and flag == 30 and lai == 0)
  {
    PIDF1();
    pidtest_time = millis();
  } else if (flag == 30) {
    if (millis() - pidtest_time < 1000) {
      Motor_reset();
      lai = 1;
    } else {

      flag++;
      pidtest_time = millis();
      lai = 0;
      servo_reset();
      digitalWrite(angle90, LOW);
    }
  }
}
