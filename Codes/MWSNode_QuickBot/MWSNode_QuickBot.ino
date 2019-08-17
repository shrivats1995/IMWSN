#include <NewPing.h>
#include <Wire.h>
#include <limits.h>
#define US1_TRIG 25
#define US1_ECHO 24
#define US2_TRIG 27
#define US2_ECHO 26
#define US3_TRIG 29
#define US3_ECHO 28
#define US4_TRIG 31
#define US4_ECHO 30
#define US5_TRIG 1
#define US5_ECHO 0
#define MTR_L_EN 14
#define MTR_L_P1 3
#define MTR_L_P2 12
#define MTR_R_EN 23
#define MTR_R_P1 13
#define MTR_R_P2 15
#define nRF_CS 18
#define nRF_SS 4
#define nRF_INT 2
#define ENC_R 11
#define ENC_L 10
#define MAG_ADDRESS 0x1E
#define MAX_DISTANCE 40
#define PI2 2*PI
#define KP_L 0.0008
#define KI_L 0
#define KD_L 0
#define KP_R 0.0008
#define KI_R 0
#define KD_R 0
#define TICK_DISTANCE_uM 7333.33
//define ultrasonic sensor objects
NewPing US1(US1_TRIG, US1_ECHO, MAX_DISTANCE);
NewPing US2(US2_TRIG, US2_ECHO, MAX_DISTANCE);
NewPing US3(US3_TRIG, US3_ECHO, MAX_DISTANCE);
NewPing US4(US4_TRIG, US4_ECHO, MAX_DISTANCE);
NewPing US5(US5_TRIG, US5_ECHO, MAX_DISTANCE);

uint8_t US1_D, US2_D, US3_D, US4_D, US5_D;
float heading;
float vel_l=0.4, vel_r=0.4;
int reqTickTime_l = 10000, reqTickTime_r = 10000;
int prevTime_l=0, prevTime_r=0;
double diffTime_l=0, diffTime_r=0;
double ePrev_l = 0,eCurr_l = 0, eCumu_l = 0, eDiff_l = 0;
double ePrev_r = 0,eCurr_r = 0, eCumu_r = 0, eDiff_r = 0;
float currPWM_l = 1, currPWM_r = 1;
boolean dBitL = false, dBitR = false; 
void init_Serial()
{
  Serial.begin(250000);
}

void init_Motors()
{
  pinMode(MTR_L_EN, OUTPUT);
  pinMode(MTR_L_P1, OUTPUT);
  pinMode(MTR_L_P2, OUTPUT);
  pinMode(MTR_R_EN, OUTPUT);
  pinMode(MTR_R_P1, OUTPUT);
  pinMode(MTR_R_P2, OUTPUT);
}

void init_Encoders()
{
  pinMode(ENC_L, INPUT);
  pinMode(ENC_R, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_L), encL_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), encR_ISR, RISING);
}

void motor_Enable(boolean left, boolean right)
{
  digitalWrite(MTR_L_EN, left);
  digitalWrite(MTR_R_EN, right);
}

void set_MotorL_PWM(float PWM)
{
  if(PWM>=0)
  {
    analogWrite(MTR_L_P1, PWM);
    digitalWrite(MTR_L_P2, 0);
  }
  else
  {
    digitalWrite(MTR_L_P1, 0);
    analogWrite(MTR_L_P2, 0);
  }
}

void set_MotorR_PWM(float PWM)
{
  if(PWM>=0)
  {
    analogWrite(MTR_R_P1, PWM);
    digitalWrite(MTR_R_P2, 0);
  }
  else
  {
    digitalWrite(MTR_R_P1, 0);
    analogWrite(MTR_R_P2, 0);
  }
}

void motor_Stop()
{
  motor_Enable(false, false);
}

void velocityToTime(float vL, float vR)
{
  reqTickTime_l = TICK_DISTANCE_uM/vL;
  reqTickTime_r = TICK_DISTANCE_uM/vR;
}

void jumpStart()
{
  set_MotorL_PWM(255);
  set_MotorR_PWM(255);
  delay(80);
  set_MotorL_PWM(0);
  set_MotorR_PWM(0);
}

void init_Mag()
{
  Wire.begin();
  Wire.beginTransmission(MAG_ADDRESS); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
}

void get_heading()
{
  int x,y,z; //triple axis data
  //Tell the HMC5883L where to begin reading data
  //Wire.begin();
  Wire.beginTransmission(MAG_ADDRESS);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  //Read data from each axis, 2 registers per axis
  Wire.requestFrom(MAG_ADDRESS, 6);
  if(6<=Wire.available())
  {
    x = Wire.read()<<8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read()<<8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read()<<8; //Y msb
    y |= Wire.read(); //Y lsb
  }
  heading = atan2(y, x);
  if(heading < 0)
    heading += PI2;
  if(heading > PI2)
    heading -= PI2;
  Serial.print("Heading (degrees): "); 
  Serial.println(heading);
}

void get_US_distances()
{
  US1_D = US1.ping_cm();
  US5_D = US5.ping_cm();
  US2_D = US2.ping_cm();
  US4_D = US4.ping_cm();
  US3_D = US3.ping_cm();
  Serial.print("Ping 1: ");
  Serial.print(US1_D); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.print("Ping 2: ");
  Serial.print(US2_D); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.print("Ping 3: ");
  Serial.print(US3_D); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.print("Ping 4: ");
  Serial.print(US4_D); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.print("Ping 5: ");
  Serial.println(US5_D); // Send ping, get distance in cm and print result (0 = outside set distance range)
}

void encL_ISR()
{
  //noInterrupts();
  int currTime_l = micros();
  diffTime_l = currTime_l-prevTime_l;
  //Serial.println(diffTime_l);
  prevTime_l = currTime_l;
  eCurr_l = diffTime_l - reqTickTime_l;
  eDiff_l = eCurr_l - ePrev_l;
  eCumu_l = eCumu_l + eCurr_l;
  currPWM_l = currPWM_l + KP_L*eCurr_l + KI_L*eCumu_l + KD_L*eDiff_l;
  ePrev_l = eCurr_l;
  //Serial.print("ErrorLeft = ");
  Serial.println(eCurr_l);
  set_MotorL_PWM(currPWM_l);
  //Serial.println(currPWM_l);
  //interrupts();
}

void encR_ISR()
{
  //noInterrupts();
  int currTime_r = micros();
  diffTime_r = currTime_r-prevTime_r;
  prevTime_r = currTime_r;
  eCurr_r = diffTime_r - reqTickTime_r;
  eDiff_r = eCurr_r - ePrev_r;
  eCumu_r = eCumu_r + eCurr_r;
  currPWM_r = currPWM_r + KP_R*eCurr_r + KI_R*eCumu_r + KD_R*eDiff_r;
  ePrev_r = eCurr_r;
  //Serial.print("ErrorRight = ");
  //Serial.println(eCurr_r);
  set_MotorR_PWM(currPWM_r);
  //interrupts();
}

void setup()
{
  delay(2000);
  init_Serial();
  init_Mag();
  init_Motors();
  init_Encoders();
  motor_Enable(true, true);
  velocityToTime(vel_l, vel_r);
  jumpStart();
  
}
void loop()
{
  //delay(33);
  //get_US_distances();
  //get_heading();
}
