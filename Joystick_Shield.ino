
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


//Defining x axis and y axis pin
#define PIN_ANALOG_X 0
#define PIN_ANALOG_Y 1


int CNS = 7;
int CE = 8;
String msg  = "";
const byte address[6] = "nrf24l";
int x = 0,y = 0;

RF24 radio(CNS,CE); 


void setup() {
  
 Serial.begin(9600);

 //For transmitter
 radio.begin();
 radio.openWritingPipe(address);
 radio.setPALevel(RF24_PA_MIN);
 radio.stopListening();

}
 

void loop() {
  
  x = analogRead(PIN_ANALOG_X);
  y = analogRead(PIN_ANALOG_Y)

 // Print values for debugging

 Serial.print("x: ");
 Serial.println(x);
 Serial.print("y: ");
 Serial.println(y);

 String temp1 = String(x);
 String temp2 = String(y);
 msg = temp1 + " " + temp2;

 
 radio.write(msg, sizeof(msg));

 delay(500);

}
