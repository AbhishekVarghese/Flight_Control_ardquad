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




RF24 radio(8,7); 
const byte address_read[6] = "00001";
Servo m1,m2,m3,m4;   //creating variables of type servo thet will control the speed of motors

//Create different pid variables for pitch and roll, however they should be nearly same so using same values for now
double P= 0.1,I = 0.009,D = 0.0007;
double P_rate =0.1 ,I_rate=0.01,D_rate=0.0007;

double req_yaw,req_pitch,req_roll;
int throttle,hover;

double inp_roll;
double yaw_correction, pitch_correction,roll_correction,out_yaw_rate,out_pitch_rate,out_roll_rate;
double gyroXrate;
double gyroYrate;
double compPitch, compRoll;

int inp_data[5];
int motor_power[4];

PID pitchPID(&compPitch, &out_pitch_rate, &req_pitch, P,I,D, DIRECT);
PID rollPID(&compRoll, &out_roll_rate, &req_roll, P,I,D, DIRECT);



PID pitchPID_rate(&gyroXrate, &pitch_correction, &out_pitch_rate,P_rate, I_rate, D_rate, DIRECT);
PID rollPID_rate(&gyroYrate, &roll_correction, &out_roll_rate,P_rate,I_rate,D_rate, DIRECT);





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
    //conf_throttle(i);
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
   compAngleX = roll;
   compAngleY = pitch;

  //start a timer
  timer = micros();

  //-----------------------------GYRO END-------------------------------


  m1.attach(5);//atttaching moters to the pins 
  m2.attach(6); //use pins with ~ these are pulse wirdth modulation pins
  m3.attach(9);
  m4.attach(10);
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address_read);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();
  
  
  pitchPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);
  pitchPID_rate.SetMode(AUTOMATIC);
  rollPID_rate.SetMode(AUTOMATIC);
  
  pitchPID.SetOutputLimits(-180,180);
  rollPID.SetOutputLimits(-180,180);
  
  pitchPID_rate.SetOutputLimits(-180,180);
  rollPID_rate.SetOutputLimits(-180,180);

  // Restting the parameters
  req_roll = 0;
  req_pitch = 0;
  req_yaw = 0;
  throttle = 0;

  //Watiting for ignition
  //wait_for_arm();
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

  gyroXrate = GyX/131.0;
  gyroYrate = GyY/131.0; 
  compAngleX = 0.99 * (compAngleX + gyroXrate * dt) + 0.01 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.99 * (compAngleY + gyroYrate * dt) + 0.01 * pitch; 

  compRoll=-compAngleX;
  compPitch = compAngleY;
  Serial.print(compPitch);Serial.print("\t");
  Serial.print(compRoll);Serial.print(" = ");
  //---------------------------------Gyroend--------------------------------

  req_roll=0;
  req_pitch =0;
  throttle = 0;
  
  if (radio.available()) {
    radio.read(&inp_data, sizeof(inp_data));
    req_roll = inp_data[3];
    req_pitch = inp_data[2];
    req_yaw = inp_data[1];
    throttle = map(inp_data[0],0,1023,0,120);    
  }
  //else Serial.println("Not available");

  //Auto ajdjust roll,pitch and yaw

  req_roll=0;
  req_pitch =0;
  req_yaw= 0;


  pitchPID.Compute();
  rollPID.Compute();
  pitchPID_rate.Compute();
  rollPID_rate.Compute();
  throttle = 110;

  Serial.print(pitch_correction);Serial.print(" , " ); Serial.println(roll_correction);
  
  if (throttle > 100) {
    motor_power[0] =  throttle + roll_correction + pitch_correction;
    motor_power[1] =  throttle - roll_correction + pitch_correction;
    motor_power[2] =  throttle - roll_correction - pitch_correction;
    motor_power[3] =  throttle + roll_correction - pitch_correction;

  }

  else {
    Serial.println("oh great");
    motor_power[0] =  throttle;
    motor_power[1] =  throttle;
    motor_power[2] =  throttle;
    motor_power[3] =  throttle;
  }
  
  
  m1.write(motor_power[0]);
  m2.write(motor_power[1]);
  m3.write(motor_power[2]);
  m4.write(motor_power[3]);

  
  /*Serial.print(motor_power[0]);Serial.print(",");
  Serial.print(motor_power[1]);Serial.print(",");
  Serial.print(motor_power[2]);Serial.print(",");
  Serial.print(motor_power[3]);
  Serial.println("-----------------------------------------------");
  */}
