const int in1 = 52;
const int in2 = 50;
const int in3 = 48;
const int in4 = 46;
const int in5 = 44;
const int in6 = 42;
const int in7 = 40;
const int in8 = 38;


const int en1 = 2;
const int en2 = 3;
const int en3 = 4;
const int en4 = 5;
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



}

void loop() {

  Motor_init();
  Motor1_Forward(200);
  delay(2000);
  Motor_init();
  Motor2_Forward(200);
  delay(2000);
  Motor_init();
  Motor3_Forward(200);
  delay(2000);
  Motor_init();
  Motor4_Forward(200);
  delay(2000);
  Motor_init();



}
void Motor_init()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  digitalWrite(in5, LOW);
  digitalWrite(in6, LOW);
  digitalWrite(in7, LOW);
  digitalWrite(in8, LOW);
}
void Motor1_Forward(int Speed)
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(en1, Speed);
}

void Motor1_Backward(int Speed)
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(en1, Speed);
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
  analogWrite(en2, Speed);
}

void Motor2_Backward(int Speed)
{
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(en2, Speed);
}
void Motor2_Brake()
{
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
void Motor3_Forward(int Speed)
{
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  analogWrite(en3, Speed);
}
void Motor4_Forward(int Speed)
{
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
  analogWrite(en4, Speed);
}
