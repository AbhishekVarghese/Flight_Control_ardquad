#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <PID>




RF24 radio(8,7); // CNS, CE
const byte address[6] = "00001";
Servo m1,m2,m3,m4;   //creating variables of type servo thet will control the speed of moters

double s1=1,s2=2,s3=3,s4=4;//FOR NOW the directions for speed later these will be replaced with inputs from gyro
double P,I,D;

double yaw,pitch,roll;
int throttle,hover;

double inp_yaw,inp_pitch,inp_roll;
double out_yaw,out_pitch,out_roll;

double inp_data[5];

PID pitchPID(&inp_pitch, &out_pitch, &pitch,P,I,D, DIRECT);
PID rollPID(&inp_roll, &out_roll, &roll,P,I,D, DIRECT);


int arm{
  int hov_thrr = 0;
  for (int i =0 ; i< 180 ;i++){
    //accy will get from gyro code
    if(acc_y>9.8){
      break
    }
    hov_thrr+=1;
  }
  return hov_thrr
}

void wait_for_arm() {
  int data[5];
  while (data [4] != 1){
     if (radio.available()) {
      radio.read(&data, sizeof(data));
    } 
  }
  //call arm
  hover = arm();
}

void setup()
  {
  m1.attach(10);//atttaching moters to the pins 
  m2.attach(9); //use pins with ~ these are pulse wirdth modulation pins
  m3.attach(6);
  m4.attach(5);
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  pitchPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);

  // Restting the parameters
  roll = 0;
  ptich = 0;
  yaw = 0;
  throttle = 0;

  //Watiting for ignition
  wait_for_arm();
}

void throttle( int power) {
  //power ranges from 0 to  180
  m1.write(power);
  m2.write(power);
  m3.write(power);
  m4.write(power);
}

void pitch( double angle, int float_power) {
  int power_diff = map( abs(angle),0,22,float_power,180);
  if (angle>0){
    m1.write(float_power+power_diff);
    m2.write(float_power+ power_diff);
    m3.write(float_power -power_diff);
    m4.write(float_power -power_diff);
  }

  else if (angle<0) {
    m1.write(float_power-power_diff);
    m2.write(float_power- power_diff);
    m3.write(float_power +power_diff);
    m4.write(float_power + power_diff);

  }
}


void roll( double angle, int float_power) {
  int power_diff = map( abs(angle),0,22,float_power,180);
  if (angle>0){
    m3.write(float_power+power_diff);
    m2.write(float_power+ power_diff);
    m1.write(float_power -power_diff);
    m4.write(float_power -power_diff);
  }

  else if (angle<0) {
    m3.write(float_power-power_diff);
    m2.write(float_power- power_diff);
    m1.write(float_power +power_diff);
    m4.write(float_power + power_diff);

  }
}


void loop()
  {
   
  if (radio.available()) {
    radio.read(&inp_data, sizeof(data));
    roll = inp_data[0];
    pitch = inp_data[1];
    yaw = inp_data[2];
    throttle = inp_data[3];
    
  }
  else Serial.println("Not available");

  //Auto ajdjust roll,pitch and yaw

  pitchPID.compute();
  rollPID.compute();

  pitch(out_pitch,hover);
  roll(out_roll,hover);
  

   }

