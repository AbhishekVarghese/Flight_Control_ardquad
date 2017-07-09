#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7,8); 
const byte address[6] = "000001";
const byte address_read[6] = "000002";


int Throttle = A0;
int Yaw      = A1;
int Pitch    = A2;  //forward bend is +ive pitch
int Roll     = A3;  // right roll is +ive


int throttle= 0;
int yaw     = 0;
int pitch   = 0;
int roll    = 0;

int colors[] = {3,5,4};//Red,green,blue

const int butn_right = 10;
const int butn_left = 9;


int msg[5];

void setup() {
  Serial.begin(9600);
  radio.begin();

  radio.openWritingPipe( address);
  radio.openReadingPipe(0, address_read);
  
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
  
  pinMode(Throttle,INPUT);
  pinMode(Yaw,INPUT); 
  pinMode(Pitch,INPUT); 
  pinMode(Roll,INPUT);

  
   //pinMode(butn_right,INPUT); 
   //pinMode(butn_left,INPUT);
  
  for (int i = 0;i <3 ; i++){
    pinMode(colors[i],OUTPUT);
  }
 
  
  msg[4] = 1;
  digitalWrite(colors[0],HIGH);

  
 
    Serial.println(digitalRead(butn_left));
   //while(digitalRead(butn_left) != HIGH){};
    Serial.println("left button pressed");
    read_copter();

  
  for(int i=0;i<3; i++){
      digitalWrite(colors[i],LOW);
    }

  wait_for_arm();
  arm_it();

  for (int i=0 ; i<20; i++) {

    for(int i=0;i<3; i++){
      digitalWrite(colors[i],HIGH);
      delay(100);
      digitalWrite(colors[i],LOW);
    }

  }

  for(int i=0;i<3; i++){
      digitalWrite(colors[i],HIGH);
    }

}





void read_copter(){
  int to_send = 32;
  int recieved = 20;
  int count = 0;
  bool failed = false;
  int while_run_time = 0;
  bool is_red = 1;
  Serial.println("insideread copter");
  for(int i =0;i<20;i++){
    
      if(radio.write(&to_send, sizeof(to_send))){
        Serial.println(i);
      }
      if(is_red){
        digitalWrite(colors[0],LOW);
        digitalWrite(colors[2],HIGH);
      }

      else{
        digitalWrite(colors[0],HIGH);
        digitalWrite(colors[2],LOW);
      }
      is_red = !is_red;
    }
        
    
  

  Serial.println("while 1 passed");
  radio.startListening();
  count = 0;
  while_run_time = 0;
  while(true){
    while_run_time ++;
    if (radio.available()){
      radio.read(&recieved,sizeof(recieved));
      if (recieved == 16){
        count ++;
      }

      if (count > 20) break;
    }
    if (while_run_time > 10000){
      failed = true;
      break;
    }

     if (while_run_time%2==0){
      if(is_red){
        digitalWrite(colors[0],LOW);
        digitalWrite(colors[2],HIGH);
      }

      else{
        digitalWrite(colors[0],HIGH);
        digitalWrite(colors[2],LOW);
      }
      is_red = !is_red;
    }
  }
  Serial.println("while2 passed");

}

void wait_for_arm(){
  Serial.println("waiting for armin");
  int counter = 0;
  bool on = false;
  radio.stopListening();
  while(true){
    if(digitalRead(butn_right)== HIGH){
      int msg = 4;
      Serial.println("sending arming message");
      if(radio.write(&msg,sizeof(msg))){
        Serial.println(" Arming message sent");
         break;
        }
     
    }
    

    if(counter % 10==0){
      if( !on){
        digitalWrite(colors[0],HIGH);
      }
      else{
        digitalWrite(colors[0],LOW);
      }
      counter = 0;
    }
    counter++;   
  }

  Serial.println("passed");
}

void arm_it(){
  digitalWrite(colors[0],LOW);
  digitalWrite(colors[1],HIGH);
  int msg = 4;
  radio.startListening();
  while(true){
    if(radio.available()){
      radio.read(&msg,sizeof(msg));
      if (msg == 1)break;
    }
  }
  digitalWrite(colors[1],LOW);
  radio.stopListening();
  Serial.println("armed");
}
  


void loop() {
  
  throttle = analogRead(Throttle);
  yaw      = analogRead(Yaw);
  pitch    = analogRead(Pitch);
  roll     = analogRead(Roll);

  msg[0]= throttle;
  msg[1] = yaw;
  msg[2] = pitch;
  msg[3] = roll;

  
  radio.write(&msg, sizeof(msg));

  Serial.print(msg[0]);Serial.print(',');
  Serial.print(msg[1]);Serial.print(',');
  Serial.print(msg[2]);Serial.print(',');
  Serial.print(msg[3]);Serial.println();
  
}
