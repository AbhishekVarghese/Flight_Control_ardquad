#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
RF24 radio(8,7); // CNS, CE

int text[5] ;
int throttle;
const byte address[6] = "00001";
Servo m1,m2,m3,m4;
void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  m1.attach(5);
  m2.attach(6);
  m3.attach(9);
  m4.attach(10);
}
void loop() {
  if (radio.available()) {    
    radio.read(&text, sizeof(text));
    Serial.print(text[0]);
    Serial.print('-');
    throttle = map(text[0],0,1022,0,180);
    Serial.println(throttle);
    m1.write(throttle);
    m2.write(throttle);
    m3.write(throttle);
    m4.write(throttle);
    delay(100);
  }
  else Serial.println("Not available");
}
