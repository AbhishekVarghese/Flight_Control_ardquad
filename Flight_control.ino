#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <PID_v1.h>

// -----------------------GYRO ---------------------
#include<Wire.h>
const int MPU_addr=0x68;
double AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; 
uint32_t timer; 
double compAngleX, compAngleY;
#define degconvert 57.2957786 

//----------------------GYRO END-------------------




RF24 radio(7,8); 
const byte address[6] = "00001";
Servo m1,m2,m3,m4;   //creating variables of type servo thet will control the speed of moters


double P,I,D;

double req_yaw,req_pitch,req_roll;
int throttle,hover;

double inp_roll;
double out_yaw,out_pitch,out_roll;

double inp_data[5];
int motor_power[4];


PID pitchPID(&compAngleY, &out_pitch, &req_pitch,P,I,D, DIRECT);
PID rollPID(&compAngleX, &out_roll, &req_roll,P,I,D, DIRECT);


int arm(){
  int hov_throttle = 20;
   Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  int prev_acc = AcY;
  for (int i =hov_throttle ; i< 180 ;i++){
    //accy will get from gyro code
     Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    if(abs(AcY- prev_acc)>500){
      break;
    }
    conf_throttle(i);
    hov_throttle+=1;
    delay(1000);
  }
  return hov_throttle;
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

    //-----------------------GYRO -------------------------------

     Wire.begin();
  #if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  #else
    TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
  #endif

  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
  delay(100);

  //setup starting angle
  //1) collect the data
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  //2) calculate pitch and roll
  double roll = atan2(AcY, AcZ)*degconvert;
  double pitch = atan2(-AcX, AcZ)*degconvert;

  //3) set the starting angle to this pitch and roll
  double gyroXangle = roll;
  double gyroYangle = pitch;
  double compAngleX = roll;
  double compAngleY = pitch;

  //start a timer
  timer = micros();

  //-----------------------------GYRO END-------------------------------


  m1.attach(5);//atttaching moters to the pins 
  m2.attach(6); //use pins with ~ these are pulse wirdth modulation pins
  m3.attach(9);
  m4.attach(10);
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  pitchPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);

  // Restting the parameters
  req_roll = 0;
  req_pitch = 0;
  req_yaw = 0;
  throttle = 0;

  //Watiting for ignition
  wait_for_arm();
}

void conf_throttle( int power) {
  //power ranges from 0 to  180
  m1.write(power);
  m2.write(power);
  m3.write(power);
  m4.write(power);
}

void conf_pitch( double angle) {
  
  angle = angle-compAngleY ;
  int increment = map(abs(angle),0,20,hover,180);
  if (angle<0){
    m1.write(hover + increment);
    m2.write(hover+increment);
    m3.write(hover-increment);
    m4.write(hover-increment);
  }

  else if (angle>0) {
    m1.write(hover - increment);
    m2.write(hover-increment);
    m3.write(hover+increment);
    m4.write(hover+increment);
  }
}


void conf_roll( double angle) {
  
  angle = angle-compAngleX ;
 int  increment = map(abs(angle),0,20,hover,180);
  if (angle<0){
    m1.write(hover - increment);
    m2.write(hover+increment);
    m3.write(hover+increment);
    m4.write(hover-increment);
  }

  else if (angle>0) {
    m1.write(hover + increment);
    m2.write(hover-increment);
    m3.write(hover-increment);
    m4.write(hover+increment);
  }
}


void loop()
  {
    //--------------GYRO ----------------------------
     Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
  double dt = (double)(micros() - timer) / 1000000; //This line does three things: 1) stops the timer, 2)converts the timer's output to seconds from microseconds, 3)casts the value as a double saved to "dt".
  timer = micros(); //start the timer again so that we can calculate the next dt.

  double roll = atan2(AcY, AcZ)*degconvert;
  double pitch = atan2(-AcX, AcZ)*degconvert;

  double gyroXrate = GyX/131.0;
  double gyroYrate = GyY/131.0; 
  compAngleX = 0.99 * (compAngleX + gyroXrate * dt) + 0.01 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.99 * (compAngleY + gyroYrate * dt) + 0.01 * pitch; 
  Serial.print(compAngleX);Serial.print("\t");
  Serial.print(compAngleY);Serial.print("\n");
  //---------------------------------Gyroend--------------------------------
   
  if (radio.available()) {
    radio.read(&inp_data, sizeof(inp_data));
    req_roll = inp_data[0];
    req_pitch = inp_data[1];
    req_yaw = inp_data[2];
    throttle = inp_data[3];
    
  }
  else Serial.println("Not available");

  //Auto ajdjust roll,pitch and yaw

  pitchPID.Compute();
  rollPID.Compute();

  conf_pitch(out_pitch);
  conf_roll(out_roll);
  
  m1.write(motor_power[0]);
  m2.write(motor_power[1]);
  m3.write(motor_power[2]);
  m4.write(motor_power[3]);

   }

