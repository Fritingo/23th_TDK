#include<Wire.h>

#define KS103_1 0x74
#define KS103_2 0x75
#define DATA_SIZE 8

word distance=0;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println("I2C Master started");
  Serial.println();

  setting_ks103(KS103_1, 0x71);
  setting_ks103(KS103_2, 0x71);
}

void loop() {

  KS103_read(KS103_1);
  Serial.print("第一顆=");
  Serial.print(distance); 
  Serial.print("mm");
  delay(100);
  KS103_read(KS103_2);
  Serial.print(" 第二顆=");
  Serial.print(distance); 
  Serial.println("mm"); 
  delay(100);
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

word KS103_read(int addr){ 
  Wire.beginTransmission(addr);
  Wire.write(byte(0x02));     
  Wire.write(0xb0);     //量程设置为5m 不带温度补偿
  Wire.endTransmission();     
  delay(1);                
  Wire.beginTransmission(addr); 
  Wire.write(byte(0x02));    
  Wire.endTransmission();   
  Wire.requestFrom(addr, 2);  
  if(2 <= Wire.available())   
  {
    distance = Wire.read(); 
    distance =  distance << 8;   
    distance |= Wire.read();
  }
}
