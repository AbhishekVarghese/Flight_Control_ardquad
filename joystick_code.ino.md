#include<SPI.h>
#include "nRF24L01.h" 
#include "RF24.h"

#define cepin 7
#define csnpin 4



#define Xpin1 A0 
#define Ypin1 A1
#define Xpin2 A2
#define Ypin2 A3


//Map SPI port to 'new' pins D14..D17
static const uint8_t MOSI = 5;
static const uint8_t MISO = 9;
static const uint8_t SCK  = 8;



const uint64_t pipe = 0xE1E2E3E4E5LL;

RF24 radio(cepin,csnpin);

double joystick[4];
int maxangle = 22;

void setup() 
{ 
  Serial.begin(115200); 
  radio.begin(); 
  radio.openWritingPipe(pipe);

}

void loop()
{ 
  joystick[0]= analogRead(Xpin1); 
  joystick[1]= analogRead(Ypin1);

  joystick[2]= digitalRead(Xpin2);
  joystick[3]= digitalRead(Ypin2);

  joystick[0] = map(joystick[0],0,1024,-maxangle,maxangle);
  joystick[1] = map(joystick[1],0,1024,-maxangle,maxangle);
  joystick[2] = map(joystick[2],0,1024,-maxangle,maxangle);
  joystick[3] = map(joystick[3],0,1024,-maxangle,maxangle);

  radio.write(joystick,sizeof(joystick));
}

  
