int t1 = 0;
int t2 = 0;
boolean dbitR = false;
boolean dbitL = false;
#define ReqTime 10000
#define ReqSpeedL 50
#define ReqSpeedR 50
#define Kp 1
#define Ki 0
#define Kd 0
byte encR = 0;
byte encL = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000);
  pinMode(23, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(15, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(11, INPUT);
  pinMode(10, INPUT);
  digitalWrite(23, 1);
  digitalWrite(14 , 1);
  //analogWrite(13, 155);
  digitalWrite(15, 0);
  digitalWrite(12, 0);
  attachInterrupt(digitalPinToInterrupt(10), encoderR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(11), encoderL, CHANGE);
  //jump();
}

float sArray_l[8] = {0};
float sArray_r[8] = {0};
float avgFilterL(float sample)
{
  float avg = 0;
  for (int i = 1; i < 8; i++)
  {
    avg += sArray_l[i];
    sArray_l[i - 1] = sArray_l[i];
  }
  avg += sample;
  sArray_l[7] = sample;
  avg /= 8;
  return avg;
}
float avgFilterR(float sample)
{
  float avg = 0;
  for (int i = 1; i < 8; i++)
  {
    avg += sArray_r[i];
    sArray_r[i - 1] = sArray_r[i];
  }
  avg += sample;
  sArray_r[7] = sample;
  avg /= 8;
  return avg;
}

/*void jump()
  {
  digitalWrite(13, HIGH);
  digitalWrite(3, HIGH);
  delay(50);
  digitalWrite(13, LOW);
  digitalWrite(3, LOW);
  }*/
float speedR = 0, speedL = 0;
double ePrevR = 0, eCurrR = 0, eCumuR = 0, eDR = 0;
float currPWMR = 1;
double ePrevL = 0, eCurrL = 0, eCumuL = 0, eDL = 0;
float currPWML = 1;
void loop()
{
  t2 = micros();
  //Serial.println(t2-t1);
  if (t2 - t1 >= 25000)
  {
    int p = micros();
    speedR = 14.56 * encR;
    speedL = 14.56 * encL;
    t1 = t2;
    encR = 0;
    encL = 0;
    eCurrR = ReqSpeedR - speedR;
    eCurrL = ReqSpeedL - speedL;
    eDR = eCurrR - ePrevR;
    eDL = eCurrL - ePrevL;
    eCumuR += eCurrR;
    eCumuL += eCurrL;
    currPWMR = currPWMR + Kp * eCurrR + Ki * eCumuR + Kd * eDR;
    currPWML = currPWML + Kp * eCurrL + Ki * eCumuL + Kd * eDL;
    analogWrite(3, currPWMR);
    analogWrite(13, currPWML);
    int m = micros();
    Serial.println(speedR);
  }
}

void encoderR()
{
  encR++;
}
void encoderL()
{
  encL++;
}
