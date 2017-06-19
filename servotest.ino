#include <Servo.h>

Servo m1,m2,m3,m4;   //creating variables of type servo thet will control the speed of moters

float s1=1,s2=2,s3=3,s4=4;//FOR NOW the directions for speed later these will be replaced with inputs from gyro
void setup()
  {
  m1.attach(10);//atttaching moters to the pins 
  m2.attach(9); //use pins with ~ these are pulse wirdth modulation pins
  m3.attach(6);
  m4.attach(5);
  Serial.begin(115200);
  delay(5);
  }

void loop()
  {
   while(Serial.available()==0);//just to move forward if there is input
  float d1=analogRead(s1);
  float d2=analogRead(s2);
  float d3=analogRead(s3);
  float d4=analogRead(s4);

  d1=map(d1,0,1023,0,9);// valuse from joystick are from 0-1023
  d2=map(d2,0,1023,0,9);//giving 10 different speed levels to moters
  d3=map(d3,0,1023,0,9);
  d4=map(d4,0,1023,0,9);


  m1.write(d1);//giving the esc the command to take the given speed
  m2.write(d2);
  m3.write(d3);
  m4.write(d4);

   }

