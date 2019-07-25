#include<Wire.h>

#define KS103_1 0x74
#define KS103_2 0x75
#define DATA_SIZE 8

unsigned long ks103_time;
int ks103_state = 0;

word distance = 0;
int distanceR, distanceL;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println("I2C Master started");
  Serial.println();

  setting_ks103(KS103_1, 0x71);
  setting_ks103(KS103_2, 0x71);
}

void loop() {

  ks103_read();
  Serial.print("第一顆=");
  Serial.print(distanceL);
  Serial.print("mm");
  //  ks103_read();
  Serial.print(" 第二顆=");
  Serial.print(distanceR);
  Serial.println("mm");
  //  delay(100);
  //
  //  if (Serial.available()) {
  //    Serial.println(Serial.read());
  //    change_ks103_addr(0xea);
  //  }

}

void setting_ks103(byte addr, byte command) {
  Wire.beginTransmission(addr);
  Wire.write(byte(0x02));
  Wire.write(command);   // 发送降噪指令
  Wire.endTransmission();
  delay(1000);
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

word ks103_read() {
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
      distanceL = Wire.read();
      distanceL =  distanceL << 8;
      distanceL |= Wire.read();
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
      distanceR = Wire.read();
      distanceR =  distanceR << 8;
      distanceR |= Wire.read();
      ks103_state++;
      ks103_time = millis();
    }
  } else if ((millis() - ks103_time) > 100 and ks103_state == 4) {
  ks103_state = 0;
}
}
