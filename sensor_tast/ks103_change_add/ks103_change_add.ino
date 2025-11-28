/**
 * @file ks103_change_add.ino
 * @brief Utilities for managing KS103 ultrasonic sensor I2C addresses and reading distance.
 *
 * This sketch includes functions to:
 * - Change the I2C address of a KS103 sensor.
 * - Configure the sensor (e.g., noise reduction).
 * - Read distance measurements (blocking and non-blocking state machine versions).
 */

#include<Wire.h>
#define DATA_SIZE 8

#define KS103_L 0x74
#define KS103_R 0x75

unsigned long ks103_time;
int ks103_state = 0;

word distance = 0;
int distance_R, distance_L;

/**
 * @brief Setup function.
 *
 * Initializes Wire and Serial. Performs initial sensor configuration/address change if needed.
 */
void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println("I2C Master started");
  Serial.println();

//  setting_ks103(KS103_L, 0x75);
  setting_ks103(KS103_R, 0x75);
//  change_ks103_addr(0x74, 0x75);
//  setting_ks103(KS103_L, 0xc3);
//   setting_ks103(KS103_R, 0xc3);
//  delay(2000);
//  change_ks103_addr(0x74, 0x75);

}

/**
 * @brief Main loop.
 *
 * Reads distance from KS103_R sensor and prints it to Serial.
 */
void loop() {
//  change_ks103_addr(0x74, 0x75);
//  return
//    ks103_read();
//  ks103_update();
  distance = KS103_read(KS103_R);
  Serial.print("第一顆=");
  Serial.print(distance);
  Serial.println("cm");
//  delay(100);
//  KS103_read(KS103_R);
//  Serial.print(" 第二顆=");
//  Serial.print(distance_R);
//  Serial.println("cm");

//  Serial.print(" 超音波");
//  Serial.print(" 超音波x2");
  delay(100);
//   setting_ks103(KS103_L, 0xc3);
//   setting_ks103(KS103_R, 0xc3);
}

/**
 * @brief Configures a KS103 sensor via I2C.
 *
 * @param addr I2C address.
 * @param command Command byte.
 */
void setting_ks103(byte addr, byte command) {
  Wire.beginTransmission(addr);
  Wire.write(byte(0x02));
  Wire.write(command);   // 发送降噪指令
  Wire.endTransmission();
  delay(2000);
  Serial.print("設置設備 位址=");
  Serial.print(addr, HEX);
  Serial.print(" 指令=");
  Serial.print(command, HEX);
  Serial.println(" 成功");
}

/**
 * @brief Changes the I2C address of a KS103 sensor.
 *
 * Sends a specific sequence of commands (0x9A, 0x92, 0x9E) to unlock address changing,
 * then sends the new address.
 *
 * @param old_addr Current address.
 * @param new_addr New address (7-bit format).
 */
void change_ks103_addr(byte old_addr, byte new_addr) {
  new_addr = new_addr << 1;
  delay(5);
  Wire.beginTransmission(old_addr);
  Wire.write(byte(0x02));
  Wire.write(0x9a);   // 时序1
  Wire.endTransmission();
  delay(5);
  Wire.beginTransmission(old_addr);
  Wire.write(byte(0x02));
  Wire.write(0x92);   // 时序2
  Wire.endTransmission();
  delay(5);
  Wire.beginTransmission(old_addr);
  Wire.write(byte(0x02));
  Wire.write(0x9e);   // 时序3
  Wire.endTransmission();
  delay(5);
  Wire.beginTransmission(old_addr);
  Wire.write(byte(0x02));
  Wire.write(new_addr);   // 新地址
  Wire.endTransmission();
  delay(5);
  Serial.println("Pls check the new addr!!!");
}

/**
 * @brief Blocking read of distance from KS103 sensor.
 *
 * Triggers measurement and waits for result.
 *
 * @param addr I2C address.
 * @return word Distance in centimeters.
 */
word KS103_read(int addr) {
  Wire.beginTransmission(addr);
  Wire.write(byte(0x02));
  Wire.write(0xb4);     //量程设置为5m 带温度补偿
  Wire.endTransmission();
  delay(2);
  Wire.beginTransmission(addr);
  Wire.write(byte(0x02));
  Wire.endTransmission();
  Wire.requestFrom(addr, 2);
  if (2 == Wire.available())
  {
    distance = Wire.read();
    distance =  distance << 8;
    distance |= Wire.read();
    distance = distance / 10;
  }
  return distance;
}

/**
 * @brief Non-blocking state machine update for dual KS103 sensors.
 *
 * Cycles through triggering and reading Left and Right sensors.
 * Updates `distance_L` and `distance_R` globals.
 */
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
    if (2 == Wire.available()) {
      distance_L = Wire.read();
      distance_L =  distance_L << 8;
      distance_L |= Wire.read();
      distance_L = distance_L / 10;
      ks103_state++;
    }
    ks103_time = millis();
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
    if (2 == Wire.available()) {
      distance_R = Wire.read();
      distance_R =  distance_R << 8;
      distance_R |= Wire.read();
      distance_R = distance_R / 10;
      ks103_state++;
    }
    ks103_time = millis();
  } else if ((millis() - ks103_time) > 100 and ks103_state == 4) {
    ks103_state = 0;
  }
}

/**
 * @brief Another non-blocking state machine for reading sensors.
 *
 * Similar to `ks103_update` but uses different timing logic.
 * Updates `distance_L` and `distance_R`.
 */
void ks103_read() {
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
      ks103_state++;
      ks103_time = millis();
    }
  } else if ((millis() - ks103_time) > 100 and ks103_state == 4) {
    ks103_state = 0;
  }
}
