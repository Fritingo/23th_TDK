#include<Wire.h>

#define KS103_1 0x74
#define KS103_2 0x75
#define DATA_SIZE 8

#define KS103_L 0x74
#define KS103_R 0x75

unsigned long ks103_time;
int ks103_state = 0;

word distance = 0;
int distance_R, distance_L;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println("I2C Master started");
  Serial.println();

  setting_ks103(KS103_1, 0x75);
  setting_ks103(KS103_2, 0x75);
//  setting_ks103(KS103_1, 0xc3);
//   setting_ks103(KS103_2, 0xc3);
}

void loop() {
//    ks103_read();
  ks103_update();
//  KS103_read(KS103_1);
  Serial.print("第一顆=");
  Serial.print(distance_L);
  Serial.print("cm");
//  delay(100);
  //  ks103_read();
//  KS103_read(KS103_2);
  Serial.print(" 第二顆=");
  Serial.print(distance_R);
  Serial.println("cm");
  
  Serial.print(" 我操你超音波");
  Serial.print(" 我操你超音波x2");
  delay(100);
//   setting_ks103(KS103_1, 0xc3);
//   setting_ks103(KS103_2, 0xc3);
}

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

void change_ks103_addr(byte old_addr, byte new_addr) {
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
}

word KS103_read(int addr) {
  Wire.beginTransmission(addr);
  Wire.write(byte(0x02));
  Wire.write(0xb4);     //量程设置为5m 带温度补偿
  Wire.endTransmission();
  delay(1);
  Wire.beginTransmission(addr);
  Wire.write(byte(0x02));
  Wire.endTransmission();
  Wire.requestFrom(addr, 2);
  if (2 <= Wire.available())
  {
    distance = Wire.read();
    distance =  distance << 8;
    distance |= Wire.read();
  }
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


void ks103_read() {
  if (ks103_state == 0) {
    Wire.beginTransmission(KS103_1);
    Wire.write(byte(0x02));
    Wire.write(0xb4);     //量程设置为5m 带温度补偿
    Wire.endTransmission();
    ks103_time = millis();
    ks103_state++;
  } else if ((millis() - ks103_time) > 1 and ks103_state == 1) {
    Wire.beginTransmission(KS103_1);
    Wire.write(byte(0x02));
    Wire.endTransmission();
    Wire.requestFrom(KS103_1, 2);
    if (2 <= Wire.available()) {
      distance_L = Wire.read();
      distance_L =  distance_L << 8;
      distance_L |= Wire.read();
      ks103_time = millis();
      ks103_state++;
    }
  } else if ((millis() - ks103_time) > 100 and ks103_state == 2) {
    Wire.beginTransmission(KS103_2);
    Wire.write(byte(0x02));
    Wire.write(0xb4);     //量程设置为5m 带温度补偿
    Wire.endTransmission();
    ks103_time = millis();
    ks103_state++;
  } else if ((millis() - ks103_time) > 1 and ks103_state == 3) {
    Wire.beginTransmission(KS103_2);
    Wire.write(byte(0x02));
    Wire.endTransmission();
    Wire.requestFrom(KS103_2, 2);
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
