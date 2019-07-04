#include "MeOrion.h"
#include <Wire.h>
float orignal_x;
float orignal_y;
float orignal_z;
MeGyro gyro;
void setup()
{
  Serial.begin(115200);
  gyro.begin();
  gyro.update();
  Serial.read();

  orignal_z = gyro.getAngleZ();
}

void loop()
{
  gyro.update();
  Serial.read();
  Serial.print("orignal_z:");
  Serial.println(orignal_z);
  Serial.print(" Z:");
  Serial.println(gyro.getAngleZ() );
  Serial.print(" end:");
  Serial.println(orignal_z-gyro.getAngleZ() );
  delay(10);
}
