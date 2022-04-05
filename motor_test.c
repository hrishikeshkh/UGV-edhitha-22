const int m1p1=8;
const int m1p2=12;
const int m2p1=10;
const int m2p2=11;

void setup() {
  // put your setup code here, to run once:
pinMode(8, OUTPUT);
pinMode(12, OUTPUT);
pinMode(10, OUTPUT);
pinMode(11, OUTPUT);
pinmode(21, OUTPUT);
pinmode(22, OUTPUT);   
pinmode(25, OUTPUT);
pinmode(26, OUTPUT);
}

void loop() {
 

// m1p1=forward, m1p2= backward m1 is the right motor
//m2p1=backward,m2p2=forward  m2 is the left motor
analogWrite(21,1);
analogWrite(22,1);  
forward(5000);
delay(1000);
backward(5000);
delay(1000);
right(5000);
delay(1000);
left(5000);
delay(1000);
forward(-1);
stop_motor();
delay(10000);



}

void forward(int duration)
{
  digitalWrite(m1p1,HIGH);
  digitalWrite(m1p2,LOW);
  digitalWrite(m2p1,LOW);
  digitalWrite(m2p2,HIGH);
  if (duration !=-1){delay(duration);}
}
void backward(int duration)
{
  digitalWrite(m1p1,LOW);
  digitalWrite(m1p2,HIGH);
  digitalWrite(m2p1,HIGH);
  digitalWrite(m2p2,LOW);
  if (duration !=-1){delay(duration);}
};

void right(int duration)
{
  digitalWrite(m1p1,LOW);
  digitalWrite(m1p2,LOW);
  digitalWrite(m2p1,LOW);
  digitalWrite(m2p2,HIGH);
  if (duration !=-1){delay(duration);}
}
void left(int duration)
{
  digitalWrite(m1p1,HIGH);
  digitalWrite(m1p2,LOW);
  digitalWrite(m2p1,LOW);
  digitalWrite(m2p2,LOW);
  if (duration !=-1){delay(duration);}
}
void stop_motor()
{
  digitalWrite(m1p1,LOW);
  digitalWrite(m1p2,LOW);
  digitalWrite(m2p1,LOW);
  digitalWrite(m2p2,LOW);
  
}
