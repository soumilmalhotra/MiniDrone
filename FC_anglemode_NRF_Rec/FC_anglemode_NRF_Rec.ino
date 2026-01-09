#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


// NRF Configuration
RF24 radio(7, 10); // CE, CSN
const byte address[6] = "00001";

//IMU
#define MPU_ADDR 0x68 
float prevRateRoll, prevRatePitch, prevRateYaw;
float RateRoll=0,RatePitch=0,RateYaw=0;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
int16_t GyroX , GyroY, GyroZ,AccXLSB,AccYLSB,AccZLSB;
int RateCalibrationNumber;

//RATE CONTROLLER
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PRateRoll=0.6; float PRatePitch=PRateRoll; float PRateYaw=2.0;
float IRateRoll=1.0; float IRatePitch=IRateRoll; float IRateYaw=2.0;
float DRateRoll=0.03; float DRatePitch=DRateRoll; float DRateYaw=0;
float DesiredRateRoll, DesiredRatePitch,DesiredRateYaw;
float DesiredAngleRoll, DesiredAnglePitch;
float PIDReturn[]={0, 0, 0};

//RECIVER

struct DataPacket {
  int yaw;//yaw
  int throttle;//throttle
  int roll;//roll
  int pitch;//pitch
  // int encoderVal;
  // bool encoderBtn;
};
DataPacket receivedData ;
unsigned long lastRxTime = 0;
 
float InputRoll, InputThrottle, InputPitch, InputYaw;

//BATTERY
// float Voltage, Current, BatteryRemaining, BatteryAtStart;
// float CurrentConsumed=0;
// float BatteryDefault=1300;

//TIMING
uint32_t LoopTimer;
unsigned long prevTime=0;
float dt;

// ---------- Motor Control ----------
int motor1_output, motor2_output, motor3_output, motor4_output;
#define MOTOR_IDLE 40

//kalman filter necessaries
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};

//angle controller
float PrevErrorAngleRoll, PrevErrorAnglePitch;
float ErrorAngleRoll, ErrorAnglePitch;
float PrevItermAngleRoll, PrevItermAnglePitch;
float PAngleRoll=2; float PAnglePitch=PAngleRoll;
float IAngleRoll=0; float IAnglePitch=IAngleRoll;
float DAngleRoll=0; float DAnglePitch=DAngleRoll;

//-----------------------------------------------------------FUNCTIONS----------------------------------------------------------------------- 




void kalman_1d(float &KalmanState, float &KalmanUncertainty, float KalmanInput, float KalmanMeasurement, float dt) {
  KalmanState=KalmanState+dt*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + dt * dt * 4.0f * 4.0f;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3.0f * 3.0f);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1.0f - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}
// void battery_voltage(void) {
//   Voltage=(float)analogRead(15)/62;
//   Current=(float)analogRead(21)*0.089;
// }
void read_receiver(void){
    if (radio.available()) {
      radio.read(&receivedData, sizeof(receivedData));
      lastRxTime = millis();

    }
}
void gyro_signals(void) {
//LOW_PASS FILTER already in the setup
  // Wire.beginTransmission(MPU_ADDR);
  // Wire.write(0x1A);
  // Wire.write(0x05);
  // Wire.endTransmission();

  //ACCMETER CONFIG already in setup 
  // Wire.beginTransmission(MPU_ADDR);
  // Wire.write(0x1C);
  // Wire.write(0x10);
  // Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(MPU_ADDR,6);
  if (Wire.available() >= 6) {
     AccXLSB = Wire.read() << 8 | Wire.read();
     AccYLSB = Wire.read() << 8 | Wire.read();
     AccZLSB = Wire.read() << 8 | Wire.read();
  }
  //GYRO CONFIG alr in setup
  // Wire.beginTransmission(MPU_ADDR);
  // Wire.write(0x1B); 
  // Wire.write(0x8);
  // Wire.endTransmission();   


  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADDR,6);
  if (Wire.available() >= 6) {
     GyroX = Wire.read() << 8 | Wire.read();
     GyroY = Wire.read() << 8 | Wire.read();
     GyroZ = Wire.read() << 8 | Wire.read();
  }

  // //rate roll values
  // float gyroScale = 65.5;  // for ±500 dps
  // float newRateRoll  = GyroX / gyroScale;
  // float newRatePitch = GyroY / gyroScale;
  // float newRateYaw   = GyroZ / gyroScale;

  // // Apply simple low-pass filter
  // RateRoll  = 0.9 * RateRoll + 0.1 * newRateRoll;
  // RatePitch = 0.9 * RatePitch + 0.1 * newRatePitch;
  // RateYaw   = 0.9 * RateYaw + 0.1 * newRateYaw;

  RateRoll = (float)GyroX / 65.5f;   // ±500 dps sensitivity
  RatePitch = (float)GyroY / 65.5f;
  RateYaw = (float)GyroZ / 65.5f;  
  
  
  //acc values
  AccX=(float)AccXLSB/4096;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096;


  //Angles using trignometery  
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))* 57.29578f;
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))* 57.29578f;
}


//PID EQN
void pid_equation(float Error, float P , float I, float D, float PrevError, float PrevIterm ,float dt) {
  //p term calc
  float Pterm=P*Error;

  //i term calc
  if (dt <= 0.00001f) dt = 0.00001f;
  float Iterm=PrevIterm+I*(Error+PrevError)*dt/2;
  Iterm = constrain(Iterm,-60, 60);
  
  //d term calc
  float Dterm=D*(Error-PrevError)/dt;
  
  //output calc
  float PIDOutput= Pterm+Iterm+Dterm;
  
  //output constrain
  PIDOutput = constrain(PIDOutput, -60, 60);
  
  PIDReturn[0]=PIDOutput;
  PIDReturn[1]=Error;
  PIDReturn[2]=Iterm;
}


void reset_pid(void) {
  PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
  PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;
  PrevErrorAngleRoll=0; PrevErrorAnglePitch=0;    
  PrevItermAngleRoll=0; PrevItermAnglePitch=0;
}

//-----------------------------------------------SETUP------------------------------------------

void setup() {
  Serial.begin(115200);
  Wire.setClock(400000);
  Wire.begin();

  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.openReadingPipe(1, address);
  radio.startListening();

  //MOTOR WIRING: 3,5,6,9
  // WOULD ADD A BUZZER HERE

  delay(250);
 
    // Check MPU connection
  // Wire.beginTransmission(MPU_ADDR);
  // if (Wire.endTransmission() != 0) {
  //   Serial.println("MPU6050 not detected!");
  //   while (1);
  // }   
  //Wake up mpu6500
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

    // Configure DLPF (low-pass)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A); Wire.write(0x05); // 44Hz cutoff
  Wire.endTransmission();
    // Configure gyro
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); Wire.write(0x08); // ±500 deg/s
  Wire.endTransmission();
  //ACCMETER CONFIG  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  
  delay(20); // just a small delay before calibration starts


  for (RateCalibrationNumber=0; 
        RateCalibrationNumber<2000;
        RateCalibrationNumber ++) {
          gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
  }
  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;



  // battery_voltage();
  // if (Voltage > 8.3) { digitalWrite(5, LOW); BatteryAtStart=BatteryDefault; }
  // else if (Voltage < 7.5) {BatteryAtStart=30/100*BatteryDefault ;}
  // else { digitalWrite(5, LOW); BatteryAtStart=(82*Voltage-580)/100*BatteryDefault; }
  // ReceiverInput.begin(14);
  // while (ReceiverValue[2] < 1020 || 
  //        ReceiverValue[2] > 1050) {
  //   read_receiver();
  //   delay(4);
  // }

  prevTime = micros();

}
void loop() {

//getting the calibrated R P Y
  gyro_signals();
  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch; 
  RateYaw-=RateCalibrationYaw;

// SETTING THE TIME TO ACCURATLY PROVIDE ITERATION LENGTH 
  unsigned long now = micros();
  dt = (now - prevTime) / 1000000.0; // convert micro seconds to seconds
  prevTime = now;

//getting the KALMAN filtered angles
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll, dt);
  KalmanAngleRoll=Kalman1DOutput[0]; KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch, dt);
  KalmanAnglePitch=Kalman1DOutput[0]; KalmanUncertaintyAnglePitch=Kalman1DOutput[1];

//getting the desired angles 
  read_receiver();
   
   //failsafe
  // if (millis() - lastRxTime > 300) {   // 300 ms timeout
  //   motor1_output = 0;
  //   motor2_output = 0;
  //   motor3_output = 0;
  //   motor4_output = 0;
  //   reset_pid();
  //   return;
  // }

  receivedData.throttle = constrain(receivedData.throttle, 0, 1023);
  receivedData.roll     = constrain(receivedData.roll, 0, 1023);
  receivedData.pitch    = constrain(receivedData.pitch, 0, 1023);
  receivedData.yaw      = constrain(receivedData.yaw, 0, 1023);

  DesiredAngleRoll=map(receivedData.roll,0,1023,-20,20);
  DesiredAnglePitch=map(receivedData.pitch,0,1023,-20,20); 
  InputThrottle= map(receivedData.throttle,0,1023,0,225);
  DesiredRateYaw=map(receivedData.yaw,0,1023,-90,90); 

// ANGLE controller, aka PID equation to smoothly convert angles into rotation rate in appropriate settling time and with minimum overshoot 
  ErrorAngleRoll=DesiredAngleRoll-KalmanAngleRoll;
  ErrorAnglePitch=DesiredAnglePitch-KalmanAnglePitch;

  //roll eqn
  pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll,dt);     
  DesiredRateRoll=PIDReturn[0];
  PrevErrorAngleRoll=PIDReturn[1];
  PrevItermAngleRoll=PIDReturn[2];

  //pitch eqn
  pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch, dt);
  DesiredRatePitch=PIDReturn[0]; 
  PrevErrorAnglePitch=PIDReturn[1];
  PrevItermAnglePitch=PIDReturn[2];
  

//RATE Controller, aka PID equation to smoothly convert pitch,roll,yaw rate into motor inputs with minimum overshoot and in least settling time
  ErrorRateRoll=DesiredRateRoll-RateRoll;
  ErrorRatePitch=DesiredRatePitch-RatePitch;
  ErrorRateYaw=DesiredRateYaw-RateYaw;
  //roll eqn
  pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll ,dt);
       InputRoll=PIDReturn[0];
       PrevErrorRateRoll=PIDReturn[1]; 
       PrevItermRateRoll=PIDReturn[2];
  //pitch eqn
  pid_equation(ErrorRatePitch, PRatePitch,IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch, dt);
       InputPitch=PIDReturn[0]; 
       PrevErrorRatePitch=PIDReturn[1]; 
       PrevItermRatePitch=PIDReturn[2];
  //yaw eqn
  pid_equation(ErrorRateYaw, PRateYaw,IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw ,dt);
       InputYaw=PIDReturn[0]; 
       PrevErrorRateYaw=PIDReturn[1]; 
       PrevItermRateYaw=PIDReturn[2];

  

//basic motor equations for drone
  motor1_output= (InputThrottle-InputRoll-InputPitch-InputYaw);
  motor2_output= (InputThrottle-InputRoll+InputPitch+InputYaw);
  motor3_output= (InputThrottle+InputRoll+InputPitch-InputYaw);
  motor4_output= (InputThrottle+InputRoll-InputPitch+InputYaw);



  // Constrain outputs
  motor1_output = constrain(motor1_output, 0, 225);
  motor2_output = constrain(motor2_output, 0, 225);
  motor3_output = constrain(motor3_output, 0, 225);
  motor4_output = constrain(motor4_output, 0, 225);

  if (motor1_output > 0 && motor1_output < MOTOR_IDLE) motor1_output = MOTOR_IDLE;
  if (motor2_output > 0 && motor2_output < MOTOR_IDLE) motor2_output = MOTOR_IDLE;
  if (motor3_output > 0 && motor3_output < MOTOR_IDLE) motor3_output = MOTOR_IDLE;
  if (motor4_output > 0 && motor4_output < MOTOR_IDLE) motor4_output = MOTOR_IDLE;

  int ThrottleCutOff=0;

  if (receivedData.throttle<20) {
    motor1_output=ThrottleCutOff; 
    motor2_output=ThrottleCutOff;
    motor3_output=ThrottleCutOff; 
    motor4_output=ThrottleCutOff;
    reset_pid();
  }

  // Serial.println(motor1_output);
  // Serial.println(motor2_output);
  // Serial.println(motor3_output);
  // Serial.println(motor4_output);


  analogWrite(3,motor1_output);
  analogWrite(5,motor2_output);
  analogWrite(6,motor3_output); 
  analogWrite(9,motor4_output);

  // battery_voltage();
  // CurrentConsumed=Current*1000*0.004/3600+CurrentConsumed;
  // BatteryRemaining=(BatteryAtStart-CurrentConsumed)/BatteryDefault*100;
  // if (BatteryRemaining<=30) digitalWrite(5, HIGH);
  // else digitalWrite(5, LOW);
  while (micros() - LoopTimer < 4000);
  LoopTimer=micros();
}