



int command = 0;


void setup() {
 

 Serial.begin(115200);
  Serial1.begin(9600);
   Serial.print("a");
}

void loop() {

  if ( Serial1.available() > 0)   {
    Serial.print(Serial1.read());
//    pidtest_time = millis();
    command = Serial1.read();// '85117/\''68100\/''76108<''82114>'

  }


}
