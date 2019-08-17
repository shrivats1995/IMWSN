#include <MatrixMath.h>
#include <NewPing.h>
#include <Wire.h>
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
#define MAX_DISTANCE 55
#define PI2 2*PI
#define KP_L 1
#define KI_L 0
#define KD_L 0
#define KP_R 1
#define KI_R 0
#define KD_R 0
#define KP_AO 0.4
#define KI_AO 0
#define KD_AO 0.1
#define DT 0.033
#define R 0.035
#define L 0.16
#define DIST_TICK 0.007333/2
#define MAX_VEL 25.656
#define MIN_VEL 4.28
#define MAX_W (2*R*MAX_VEL)/L
#define MIN_W (2*R*MIN_VEL)/L
//define ultrasonic sensor objects
NewPing US[5] = {NewPing(US1_TRIG, US1_ECHO, MAX_DISTANCE),
                 NewPing(US2_TRIG, US2_ECHO, MAX_DISTANCE),
                 NewPing(US3_TRIG, US3_ECHO, MAX_DISTANCE),
                 NewPing(US4_TRIG, US4_ECHO, MAX_DISTANCE),
                 NewPing(US5_TRIG, US5_ECHO, MAX_DISTANCE)
                };
float xymat[2][5];
float Rt[3][3];
float USPosition[3][5] = {{ -0.07, -0.053, 0, 0.053, 0.07}, {0, 0.053, 0.075, 0.053, 0}, {-PI / 2, -PI / 4, 0, +PI / 4, +PI / 2}};
float USDistance_rf[3][5];
float USDistance_wf[2][5];
float sensor_gains[5][5] = {{1, 0, 0, 0, 0},
                            {0, 1, 0, 0, 0},
                            {0, 0, 1, 0, 0},
                            {0, 0, 0, 1, 0},
                            {0, 0, 0, 0, 1}};
float theta_ao;
float u_i[3][5];
float u_i_intermediate[2][5];
float u_ao[2] = {0, 0};
float ep_ao = 0, ei_ao = 0, ed_ao = 0, ep_ao_prev = 0;
float x = 0, y = 0, theta = 0;
float v = 0.4, w = 0;
float vel_r_tmp, vel_l_tmp;
float vtmp, wtmp;
int t1 = 0, t2 = 0;
byte encR = 0, encL = 0, encR_prev = 0, encL_prev = 0;
float US_F[5];
float vel_l = 11.426, vel_r = 11.426;
float currSpeed_l = 0, currSpeed_r = 0;
int prevTime_l = 0, prevTime_r = 0;
double diffTime_l = 0, diffTime_r = 0;
double ePrev_l = 0, eCurr_l = 0, eCumu_l = 0, eDiff_l = 0;
double ePrev_r = 0, eCurr_r = 0, eCumu_r = 0, eDiff_r = 0;
float currPWM_l = 1, currPWM_r = 1;
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
  attachInterrupt(digitalPinToInterrupt(ENC_L), encL_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R), encR_ISR, CHANGE);
}

void motor_Enable(boolean left, boolean right)
{
  digitalWrite(MTR_L_EN, left);
  digitalWrite(MTR_R_EN, right);
}

void set_MotorL_PWM(float PWM)
{
  if (PWM >= 0)
  {
    analogWrite(MTR_L_P1, PWM);
    digitalWrite(MTR_L_P2, 0);
  }
  else
  {
    digitalWrite(MTR_L_P1, 0);
    analogWrite(MTR_L_P2, -PWM);
  }
}

void set_MotorR_PWM(float PWM)
{
  if (PWM >= 0)
  {
    analogWrite(MTR_R_P1, PWM);
    digitalWrite(MTR_R_P2, 0);
  }
  else
  {
    digitalWrite(MTR_R_P1, 0);
    analogWrite(MTR_R_P2, -PWM);
  }
}

void motor_Stop()
{
  motor_Enable(false, false);
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
  int x, y, z; //triple axis data
  Wire.beginTransmission(MAG_ADDRESS);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  //Read data from each axis, 2 registers per axis
  Wire.requestFrom(MAG_ADDRESS, 6);
  if (6 <= Wire.available())
  {
    x = Wire.read() << 8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read() << 8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read() << 8; //Y msb
    y |= Wire.read(); //Y lsb
  }
  theta = atan2(y, x);
  //if (theta < 0)
  //  theta += PI2;
  //if (theta > PI2)
  //  theta -= PI2;
  Serial.print("Heading: ");
  Serial.println(theta);
}

void get_US_distances()
{
  US_F[0] = ((float)US[0].ping_cm())/100;
  US_F[0] = US_F[0]==0?5:US_F[0];
  US_F[2] = ((float)US[2].ping_cm())/100;
  US_F[2] = US_F[2]==0?5:US_F[2];
  US_F[4] = ((float)US[4].ping_cm())/100;
  US_F[4] = US_F[0]==0?5:US_F[4];
  US_F[1] = ((float)US[1].ping_cm())/100;
  US_F[1] = US_F[1]==0?5:US_F[1];
  US_F[3] = ((float)US[3].ping_cm())/100;
  US_F[3] = US_F[3]==0?5:US_F[3];
  Serial.print("PI/2: ");
  printDouble(US_F[0],100); // Send ping, get distance in cm and print result (0 = outside set distance range)th
  Serial.print(" PI/4: ");
  printDouble(US_F[1],100); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.print(" 0: ");
  printDouble(US_F[2],100); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.print(" -PI/4: ");
  printDouble(US_F[3],100); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.print(" -PI/2: ");
  printDouble(US_F[4],100); // Send ping, get distance in cm and print result (0 = outside set distance range)
}

void printDouble( double val, unsigned int precision){
// prints val with number of decimal places determine by precision
// NOTE: precision is 1 followed by the number of zeros for the desired number of decimial places
// example: printDouble( 3.1415, 100); // prints 3.14 (two decimal places)

   Serial.print (int(val));  //prints the int part
   Serial.print("."); // print the decimal point
   unsigned int frac;
   if(val >= 0)
       frac = (val - int(val)) * precision;
   else
       frac = (int(val)- val ) * precision;
   Serial.print(frac,DEC) ;
}

void encL_ISR()
{
  encL++;
}

void encR_ISR()
{
  encR++;
}

void repmat()
{
  xymat[0][0] = x;
  xymat[0][1] = x;
  xymat[0][2] = x;
  xymat[0][3] = x;
  xymat[0][4] = x;
  xymat[1][0] = y;
  xymat[1][1] = y;
  xymat[1][2] = y;
  xymat[1][3] = y;
  xymat[1][4] = y;
}

void get_transformation_matrix(float x1, float y1, float theta1)
{
  Rt[0][0] = cos(theta1);
  Rt[0][1] = -sin(theta1);
  Rt[0][2] = x1;
  Rt[1][0] = sin(theta1);
  Rt[1][1] = cos(theta1);
  Rt[1][2] = y1;
  Rt[2][0] = 0;
  Rt[2][1] = 0;
  Rt[2][2] = 1;
}

void apply_sensor_geometry()
{
  for (int i = 0; i < 5; i++)
  {
    get_transformation_matrix(USPosition[0][i], USPosition[1][i], USPosition[2][i]);
    for (int j = 0; j < 2; j++)
    {
      USDistance_rf[j][i] = Rt[j][0] * US_F[i] + Rt[j][2];
    }
    Matrix.Multiply((float*)Rt, (float*)USDistance_rf, 3, 3, 5, (float*)USDistance_wf);
  }
}

void executeAO()
{
  apply_sensor_geometry();
  repmat();
  Matrix.Multiply((float*)xymat, (float*)sensor_gains, 2, 5, 5, (float*)u_i_intermediate);
  Matrix.Subtract((float*)USDistance_wf, (float*)u_i_intermediate, 2, 5, (float*)u_i);
  u_ao[0] = 0;
  u_ao[1] = 1;
  for (int i = 0; i < 5; i++)
  {
    u_ao[0] += u_i[0][i];
    u_ao[1] += u_i[1][i];
  }
  theta_ao = atan2(u_ao[1], u_ao[0]);
  Serial.print("Theta A0 = ");
  Serial.println(theta_ao);
  ep_ao = theta_ao - theta;
  ei_ao += ep_ao * DT;
  ed_ao = (ep_ao - ep_ao_prev) / DT;
  ep_ao_prev = ep_ao;
  w = KP_AO * ep_ao + KI_AO * ei_ao + KD_AO * ed_ao;
}

void update_odometry()
{
  float d_right = encR_prev * DIST_TICK;
  float d_left = encL_prev * DIST_TICK;
  float d_centre = (d_right + d_left) / 2;
  float x_dt = d_centre * cos(theta);
  float y_dt = d_centre * sin(theta);
  x += x_dt;
  Serial.print("X = ");
  Serial.println(x);
  y += y_dt;
  Serial.print("Y = ");
  Serial.println(y);
}

void uni_to_diff(float v1, float w1)
{
  vel_r_tmp = (2 * v1 + w1 * L) / (2 * R);
  vel_l_tmp = (2 * v1 - w1 * L) / (2 * R);
}

void diff_to_uni(float vr, float vl)
{
  vtmp = R / 2 * (vr + vl);
  wtmp = R / L * (vr - vl);
}

void ensure_w()
{
  Serial.print("Before Ensure_W V = ");
  Serial.println(v);
  Serial.print("Before Ensure_W w = ");
  Serial.println(w);
  if (abs(v) > 0)
  {
    float v_lim = max(min(abs(v), (R) * (MAX_VEL)), (R) * (MIN_VEL));
    float w_lim = max(min(abs(w), (R / L) * (MAX_VEL - MIN_VEL)), 0);
    uni_to_diff(v_lim, w_lim);
    float vel_r_d = vel_r_tmp;
    float vel_l_d = vel_l_tmp;
    float vel_rl_max = max(vel_r_d, vel_l_d);
    float vel_rl_min = min(vel_r_d, vel_l_d);
    if (vel_rl_max > MAX_VEL)
    {
      vel_r = vel_r_d - (vel_rl_max - MAX_VEL);
      vel_l = vel_l_d - (vel_rl_max - MAX_VEL);
    }
    else if (vel_rl_min < MIN_VEL)
    {
      vel_r = vel_r_d + (MIN_VEL - vel_rl_min);
      vel_l = vel_l_d + (MIN_VEL - vel_rl_min);
    }
    else
    {
      vel_r = vel_r_d;
      vel_l = vel_l_d;
    }
    diff_to_uni(vel_r, vel_l);
    float v_shift = vtmp;
    float w_shift = wtmp;
    v = v_shift >= 0 ? v_shift : -v_shift;
    w = w_shift >= 0 ? w_shift : -w_shift;
  }
  else
  {
    if (abs(w) > MIN_W)
    {
      w = w >= 0 ? max(min(abs(w), MAX_W), MIN_W) : -max(min(abs(w), MAX_W), MIN_W);
    }
    else
    {
      w = 0;
    }
  }
  Serial.print("After Ensure_W V = ");
  Serial.println(v);
  Serial.print("After Ensure_W W = ");
  Serial.println(w);
  uni_to_diff(v, w);
  vel_r = vel_r_tmp;
  vel_l = vel_l_tmp;
  Serial.print("Vel R = ");
  Serial.println(vel_r);
  Serial.print("Vel L = ");
  Serial.println(vel_l);
}

void setup()
{
  delay(2000);
  init_Serial();
  init_Mag();
  init_Motors();
  init_Encoders();
  motor_Enable(true, true);
  Serial.println("===========================");
}

void loop()
{
  t2 = millis();
  if (t2 - t1 >= 33)
  {
    //int t11 = micros();
    //int p = micros();
    currSpeed_r = 3.173 * encR;
    currSpeed_l = 3.173 * encL;
    encR_prev = encR;
    encL_prev = encL;
    encR = 0;
    encL = 0;
    t1 = t2;
    //Serial.print("Current Speed Left = ");
    //Serial.println(currSpeed_l);
    //Serial.print("Current Speed Right = ");
    Serial.println(currSpeed_r);
    eCurr_r = abs(vel_r) - currSpeed_r;
    eCurr_l = abs(vel_l) - currSpeed_l;
    eDiff_r = eCurr_r - ePrev_r;
    eDiff_l = eCurr_l - ePrev_r;
    eCumu_r += eCurr_r;
    eCumu_l += eCurr_l;
    currPWM_r += KP_R * eCurr_r + KI_R * eCumu_r + KD_R * eDiff_r;
    currPWM_l += KP_L * eCurr_l + KI_L * eCumu_l + KD_L * eDiff_l;
    set_MotorR_PWM(vel_r >= 0 ? currPWM_r : -currPWM_r);
    set_MotorL_PWM(vel_l >= 0 ? currPWM_l : -currPWM_l);
    get_US_distances();
    get_heading();
    executeAO();
    Serial.print("V = ");
    Serial.println(v);
    Serial.print("W = ");
    Serial.println(w);
    //ensure_w();
    uni_to_diff(v, w);
    vel_r = vel_r_tmp;
    vel_l = vel_l_tmp;
    Serial.print("Vel R = ");
    Serial.println(vel_r);
    Serial.print("Vel L = ");
    Serial.println(vel_l);
    update_odometry();
    Serial.println("===========================");
    //int t22 = micros();
    //Serial.print("Time = ");
    //Serial.println(t22 - t11);
  }
}
