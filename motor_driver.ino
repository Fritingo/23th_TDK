const int in1 = 52;
const int in2 = 50;
const int in3 = 48;
const int in4 = 46;

const int ena = 2;
const int enb = 3;
void setup() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);



}

void loop() {


  Motor1_Forward(200);
  delay(1000);
  //  Motor2_Forward(200);
  //  delay(1000);
  Motor_Both_Forward(250);
  delay(1000);


}

void Motor1_Forward(int Speed)
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(ena, Speed);
}

void Motor1_Backward(int Speed)
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(ena, Speed);
}
void Motor1_Brake()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}
void Motor2_Forward(int Speed)
{
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enb, Speed);
}

void Motor2_Backward(int Speed)
{
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enb, Speed);
}
void Motor2_Brake()
{
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
void Motor_Both_Forward(int Speed)
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enb, Speed);
}
