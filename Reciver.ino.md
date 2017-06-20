# joystick-reception
receiving  the values of the joystick shield


#include <nRF24L01.h>
#include <RF24.h>
#include<Servi.h>

#define cepin  8
#define csnpin 7

const uint64_t pipe 0xE1E2E3E4E5LL;

RF24 radio(cepin,csnpin);

int joystick[6];

Servo m1,m2,m3,m4;

void setup()
{
  Serial.begin(115200);
  radio.begin();
  radio.openReadingPipe(1,pipe);
  radio.startListening();
  m1.attach(10);
  m2.attach(9);
  m3.attach(6);
  m4.attach(5);

}

void loop()
{
   if ( radio.available() )
  {
   
    bool done = false;
    while (!done)
    {
      done = radio.read( joystick, sizeof(joystick) );
      int Xval = map(joystick[0],0,1023,0,180); 
      int Yval = map(joystick[1],0,1023,0,180); 
      
      int up = joystick[2];
      int right = joystick[3];
      int down = joystick[4];
      int leftt = joystick[5];




    } 
}
