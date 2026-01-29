//input from machanisum arduino : 2 digital butons for 1.paddy in. 2.put ball in silo.
//output to machanisum arduino : 2 bits silo status. 

/*
int receiveDataSilo()      make siloData = 1 after geting the data silo_preference[3], silo_status[3]  = from python
int ballPutInSilo(silo_staus)   send 2 bits to mechanisum and read 1 as correct match and 0 as wrong match.     

pose function for sick data (merged with encoder resets encoders when used)(if cc cords are in yelow silo region the activate it.)
sick pose function macking with sam.
hadware: i2c and digital buttons setup to krishna.
*/

#include <Encoder.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <math.h>
#include <utility/imumaths.h>
#include <Adafruit_ADS1X15.h>

// int pwm_limit = 50;
// int lowerPWM = 50;
int pwm_limit = 45;
int lowerPWM = 45;
int higherPWM = 60;
float stepIncrement = 5;   //50 in ms
float beforeTimeBias = 5;     // make pwm 60(lowerPWM) again earlier before actual time.
// float MiniCorection = 25;
float MiniCorection = 25; //20
float wait = 20;
// int ki_pwmRelation = map(pwm_limit,50,200,last_i,0.01);
// float kp = 2;
// float kd = 850;
// float ki = 0.005;  //0.005  //ki_pwmRelation
float kp = 1.5;
float kd = 5;
float ki = 0.01;  //0.005  //ki_pwmRelation
float rPid[3] = { 5, 10, 0.05};


#define SlaveAdr 8
#define f1 7 //
#define b1 5  //  
#define b2 3 //
#define f1d 6 //
#define b1d 4 //   
#define b2d 2 //
#define retryPin 3
#define sillo 4;
#define paddy 5;

float scale = 1;


Adafruit_BNO055 IMU = Adafruit_BNO055(-1, 0x28, &Wire2);
Adafruit_ADS1115 ads;

// Encoder yEnc(7, 8);    // Object of encoder in x axis
// Encoder xEnc(15, 14);  // Object of encoder in y axis
// Serial.print("s");
Encoder yEnc(37, 36);    // Object of encoder in x axis
Encoder xEnc(10, 11);  // Object of encoder in y axis
// Serial.print("e");

float cc[3] = { 0, 0, 0 };
float reX = 0;
float reY = 0;
float rc[5] = { 0, 0, 0 ,reX ,reY };
// float rc[4] = { 0, 0, 0 ,0 };
float rcp[3] = { 0, 0, 0 };
float buffer[3] = { 2, 2, 1.5 };
// float rPid[3] = { 18, 100000, 0.002};  //{ 4, 500, 0.005 };   {5.5, 60, 0.007};  { 6.5, 200, 0.008} , { 4.5, 90, 0.007} , { 10, 300, 0.05} { 20, 6000, 0.02}

// float rPid[3] = { 1, 0, 0.0001};

// float kp = 1.9;
// float kd = 500;
// float ki = 0.01;  //0.005  //ki_pwmRelation
float xPid[3] = {kp, kd, ki}; //{1.2, 300, 0.0002}  {1.3, 1000, 0.00015}
float zPid[3] = {kp, kd, ki};  //{ 1.95, 1200, 0.0005}  /{4.5, 1000, 0.001}
float xCorrData[4] = { 0, 0, 0, 0};
float zCorrData[4] = { 0, 0, 0, 0};
float rCorrData[4] = { 0, 0, 0, 0};
float tErr[2] = { 0, 0 };
float corr[3] = { 0, 0, 0 };
int pid_maped[3] = { 0, 0, 0 };
// int silo_preference[3] = {0, 0 ,0};
// byte silo_status[3] = {0, 0 ,0};

int silo_preference[3] = {1, 2 ,4};
byte silo_status[3] = {0, 0 ,0};

float back = 20; 
float rcs[5][2] =  { {-140, 150-back}, {-75, 150-back}, {0, 150-back}, {75, 150-back}, {140, 150-back} };  //coordinates of 5 silos
float SiloViwePoint[2] = {0,-5};
float BallViwePoint[2] = {0,0};   //{-980,480}
float sickCoor[2] = {0,0};

float gerny = 0;
float dec = 50;
float gernyS[2] = {0,0};
int rst = 0;
int coun = 0;
float errorChange = 0;
float errorArea = 0;
float errorSlope = 0;
float side = 0.6;

float xR = 0;
float zR = 0;
float xRot = 0;
float zRot = 0;
float xDist = 0;
float zDist = 0;
float encRad = 2.9;
int ang = 0;
float rotError = 0;
float vFL, vBR, vBL;
float newTime = 0;
float oldTime = 0;
float dt = 0;
float timee = 0;
float Delay = 0;
int xa = 0;
int za = 0;
int ra = 0;
int shortPid = 20;
int shortx = 0;
int shorty = 0;
//int CorectionLimit = pwm_limit;
float pointOnRamp = 330;
int siloData = 0;


byte a;
byte b_data[3] = {0, 0, 0};
byte s_data[2] = {0, 0};
int exbit;
byte depthCamera = 1;
byte siloCamera = 2;
int dir;
int yDist;
int xDis;
int kdi;
int xyz = 0;



int sortFlagRight = 1;
int sortFlagLeft = 1;
int cnt=0;
int navigator = 0; // take dicision of silo navigation or storage ball navigation and renavigate from returning to viwepoint to detect again and go. 
float ptime = 0;
float pTime = 0;
float window = 0;
int winfirst = 0;
int silo_no = 0;
int data = 0;
int data2 = 0;

float correctionX = 0;
float correctionY = 0;
byte Mechanismdata;
int firstTimeChange = 0;

int cntSilo_no = 0;
int sendstate = 0;


void Transmit(int slaveAdr, char siloState)
{
  Wire.beginTransmission(slaveAdr);
  Wire.write(siloState);
  Wire.endTransmission();
  // Serial.println("data send to slave");
}

int Request(int slaveAdr, int adrbits)
{
   Wire.requestFrom(SlaveAdr, adrbits);
   //while (Wire.available() == 0){}
  //  Serial.print("data recived from slave : ");
   while (Wire.available())
   {
   byte c = Wire.read();
   return c;
  //  Serial.print(c);
   }
   return 0;
  // Serial.println("");
}


void receiveDataBall(){
  while(Serial.available()){
    a = Serial.read();
    b_data[coun] = a;
    // Serial.print(b_data[coun]);
    // Serial.print("a");
    // Serial.print(a);
    // delay(100);
    if(coun == 2){
      xDis = b_data[0];
      dir = (b_data[1] & 2) >> 1;
      exbit = b_data[1] & 1;
      yDist = -((exbit << 8) | (b_data[2]));

      if(dir == 0){
        xDis = -xDis;
      }
      rc[0] = xDis + cc[0];
      rc[1] = yDist + cc[1];
      if (abs(rc[0]) > 150){
        rc[0] = 150*(abs(rc[0])/rc[0]);
      }
      if (abs(rc[1]) > 280){
        rc[1] = 280*(abs(rc[1])/rc[1]);
      }
      coun = 0;
      // if(navigator == 1){
      //   rc[0] = cc[0];
      //   rc[1] = cc[1];
      // }
      break;
    }
    
    coun++;
  }

}

int receiveDataSilo(){
  int k = 0;
  while(Serial.available()){
    a = Serial.read();
    s_data[k] = a;
    k++;
    // Serial.print(data[coun]);
    }
  if (k == 2){
    // silo_status[0] = (s_data[0] & 6) >> 1;
    // silo_status[1] = (s_data[0] & 24) >> 3;
    // silo_status[2] = (s_data[0] & 96) >> 5;
    // silo_preference[0] = s_data[1] & 7;
    // silo_preference[1] = (s_data[1] & 56) >> 3;
    // silo_preference[2] = ((s_data[0] & 1) << 2) | ((s_data[1] & 192) >> 6);

    // int silo_preference[3] = {1, 2 ,3};
    // byte silo_status[3] = {0, 0 ,0};

    siloData = 1;
    return 1;
  }
  return 1;
}

void rotErrorRange(){
  rotError = cc[2];
  if (rotError > 180) {
    rotError = rotError - 360;
  } else if (rotError < -180) {
    rotError = 360 + rotError;
  }
}

// void rotErrorRange() {
//   rotError = cc[2] - tc[count][2];
//   if (rotError > 180) {
//     rotError = rotError - 360;
//   }
//   else if (rotError < -180) {
//     rotError = 360 + rotError;
//   }
// }

void pose() {
  int16_t adc0;
  int16_t adc1;
  float volts0;
  float volts1;
  float sickX;
  float sickY;

  xR = xEnc.read() / 16;  //no of rotations of x encoder
  zR = yEnc.read() / 16;  //no of rotations of y encoder
  xRot = xR / 512;
  zRot = zR / 512;

  xDist = xRot * 6.28 * encRad;
  zDist = zRot * 6.28 * encRad;

  imu::Vector<3> euler = IMU.getVector(Adafruit_BNO055::VECTOR_EULER);
  // int anglee = euler.x();
  cc[2] = euler.x();
  if (cc[2] != 0) {
    cc[2] = 360 - cc[2];
  }
  // cc[0] = reX+xDist;
  // cc[1] = reY+zDist;
  // cc[0] = xDist;
  // cc[1] = zDist;

////////////

  adc0 = ads.readADC_SingleEnded(0);
  volts0 = ads.computeVolts(adc0);
  adc1=ads.readADC_SingleEnded(1);
  volts1 = ads.computeVolts(adc1);
  // Serial.println("niche");

  rotErrorRange();

  sickX = ((volts0-0.03)*856.1)/4.7;
  sickY = ((volts1-0.27)*857)/4.7;

  sickCoor[0] = (sickX - 178) * abs(cos((rotError * 0.01745))); // + sickOffset[0];
  sickCoor[1] = -( (sickY - 182) * abs(cos((rotError * 0.01745))) ); // + sickOffset[1];

  if(cc[1] > -10){
  correctionX = sickCoor[0] - xDist;
  correctionY = sickCoor[1] - zDist;
  }

  cc[0] = xDist + correctionX;
  cc[1] = zDist + correctionY;
  // cc[0] = xDist;
  // cc[1] = zDist;
  // cc[0] = 0;
  // cc[1] = 0;

////////////

}



// int area3(){
//   rc[] = 
// }

void xxPid(float error, float pid[], float prevError, float errorArea, float dt) {
  // Serial.println("error");
  if(xa == 0){
    prevError = error;     // reset for starting new pid.
    xa++;
  }
  if(abs(tErr[0])>shortPid){
    kdi = 0;
  }
  else{
    kdi = 1;
  }
  errorChange = (error - prevError) * kdi;
  xCorrData[3] = errorChange / dt;
  xCorrData[2] = (errorArea + (error * dt)) * kdi;
  xCorrData[1] = error;
  xCorrData[0] = pid[0] * error + pid[1] * xCorrData[3] + pid[2] * xCorrData[2];
  int CorectionLimit = pwm_limit;
  if(abs(xCorrData[0]) > CorectionLimit){
    xCorrData[0] = (CorectionLimit)*(abs(xCorrData[0])/xCorrData[0]);
  }

  if(abs(xCorrData[0]) < MiniCorection){
    xCorrData[0] = (MiniCorection)*(abs(xCorrData[0])/xCorrData[0]);
  }

}

void zzPid(float error, float pid[], float prevError, float errorArea, float dt) {
  // Serial.println("error");
  if(za == 0){
    prevError = error;      // reset for starting new pid.
    za++;
  }
  if(abs(tErr[1])>shortPid){
    kdi = 0;
  }
  else{
    kdi = 1;
  }
  errorChange = (error - prevError) * kdi;
  zCorrData[3] = errorChange / dt;
  zCorrData[2] = (errorArea + (error * dt)) * kdi;
  zCorrData[1] = error;
  zCorrData[0] = pid[0] * error + pid[1] * zCorrData[3] + pid[2] * zCorrData[2];
  int CorectionLimit = pwm_limit;
  if(abs(zCorrData[0]) > CorectionLimit*1.35){
    zCorrData[0] = (CorectionLimit*1.35)*(abs(zCorrData[0])/zCorrData[0]);
  }
  if(abs(zCorrData[0]) < MiniCorection*1.5){
    zCorrData[0] = (MiniCorection*1.5)*(abs(zCorrData[0])/zCorrData[0]);
  }
}

void rrPid(float error, float pid[], float prevError, float errorArea, float dt) {
  // Serial.println("error");
  if(ra == 0){
    prevError = error;     // reset for starting new pid.
    ra++;
  }
  errorChange = error - prevError;
  rCorrData[3] = errorChange / dt;
  rCorrData[2] = errorArea + (error * dt);
  rCorrData[1] = error;
  rCorrData[0] = pid[0] * error + pid[1] * rCorrData[3] + pid[2] * rCorrData[2];
  int CorectionLimit = pwm_limit;
  if(abs(rCorrData[0]) > CorectionLimit){
    rCorrData[0] = CorectionLimit*(abs(rCorrData[0])/rCorrData[0]);
  }
  if(abs(rCorrData[0]) < MiniCorection/2){
    rCorrData[0] = MiniCorection/2*(abs(rCorrData[0])/rCorrData[0]);
  }
  // Serial.println(rCorrData[0]);
}

void velocityEquations() {
  float Vx = xCorrData[0], Vz = zCorrData[0], W = rCorrData[0];
  vFL = Vx + W * 0.288;
  vBR = -(0.5*Vx + 0.57735*Vz) + W * 0.288;    // 1.732 = 0.57735
  vBL = -(0.5*Vx - 0.57735*Vz) + W * 0.288;    // 1.732 = 0.57735
  // Serial.println(String(" ") + Vx + String(" ") + Vz + String(" ") + W);
}

// void motorPwm(int vvFL, int vvBR, int vvBL) {
//   if (vvFL >= 0) {
//     analogWrite(f1, abs(vvFL));
//     digitalWrite(f1d, HIGH);
//   } else if (vvFL < 0) {
//     analogWrite(f1, abs(vvFL));
//     digitalWrite(f1d, LOW);
//   }
//   if (vvBR >= 0) {
//     analogWrite(b1, abs(vvBR));
//     digitalWrite(b1d, HIGH);
//   } else if (vvBR < 0) {
//     analogWrite(b1, abs(vvBR));
//     digitalWrite(b1d, LOW);
//   }
//   if (vvBL >= 0) {
//     analogWrite(b2, abs(vvBL));
//     digitalWrite(b2d, HIGH);
//   } else if (vvBL < 0) {
//     analogWrite(b2, abs(vvBL));
//     digitalWrite(b2d, LOW);
//   }
// }

void motorPwm(float vvFL, float vvBR, float vvBL) {
  if (vvFL >= 0) {
    analogWrite(f1, abs(vvFL));
    digitalWrite(f1d, HIGH);
  } else if (vvFL < 0) {
    analogWrite(f1, abs(vvFL));
    digitalWrite(f1d, LOW);
  }
  if (vvBR >= 0) {
    analogWrite(b1, abs(vvBR));
    digitalWrite(b1d, HIGH);
  } else if (vvBR < 0) {
    analogWrite(b1, abs(vvBR));
    digitalWrite(b1d, LOW);
  }
  if (vvBL >= 0) {
    analogWrite(b2, abs(vvBL));
    digitalWrite(b2d, LOW);
  } else if (vvBL < 0) {
    analogWrite(b2, abs(vvBL));
    digitalWrite(b2d, HIGH);
  }
  // Serial.print(vvFL);
  // Serial.print(" ");
  // Serial.print(vvBR);
  // Serial.print(" ");
  // Serial.println(vvBL);
}


void reset_pid(){
  for(int i = 0; i < 4; i++){
    xCorrData[i] = 0;
  }
  for(int i = 0; i < 4; i++){
    zCorrData[i] = 0;
  }
  for(int i = 0; i < 4; i++){
    rCorrData[i] = 0;
  }
  ra = 0;
  xa = 0;
  za = 0;
  int abc =1;
  // motorPwm(0, 0, 0);
}
void Xreset_pid(){
  // motorPwm(0, 0, 0);
  for(int i = 0; i < 4; i++){
    xCorrData[i] = 0;
  }
  xa = 0;
}
void Zreset_pid(){
  // motorPwm(0, 0, 0);
  for(int i = 0; i < 4; i++){
    zCorrData[i] = 0;
  }
  za = 0;
}

void Rreset_pid(){
  // motorPwm(0, 0, 0);
  for(int i = 0; i < 4; i++){
    rCorrData[i] = 0;
  }
  ra = 0;
}

int findGreatest(int vFL2, int vBR2, int vBL2) {
  int greatestNumber = vFL2; // Assume vFL2 is initially the greatest

  // Compare vBR2 with greatestNumber
  if (vBR2 > greatestNumber) {
    greatestNumber = vBR2;
  }

  // Compare vBL2 with greatestNumber
  if (vBL2 > greatestNumber) {
    greatestNumber = vBL2;
  }

  // Return the greatest number
  return greatestNumber;
}

void pid_map(int pwm_limit,int vFL2,int vBR2,int vBL2){
  int maxnum = findGreatest(abs(vFL2), abs(vBR2), abs(vBL2));
  pid_maped[0] = vFL2;
  pid_maped[1] = vBR2;
  pid_maped[2] = vBL2;
  if(maxnum > pwm_limit){
  pid_maped[0] = map(abs(vFL2),0,maxnum,0,pwm_limit) * (vFL2/abs(vFL2)); //vFL2
  pid_maped[1] = map(abs(vBR2),0,maxnum,0,pwm_limit) * (vBR2/abs(vBR2)); //vBR2
  pid_maped[2] = map(abs(vBL2),0,maxnum,0,pwm_limit) * (vBL2/abs(vBL2)); //vBL2
  }
}

void target_cords(int xyz){
  //targeted co-ordinates
  if (xyz == 0){
    tErr[0] = rc[0] - cc[0];  // x cor
    if (abs(tErr[0]) > buffer[0] && rst==0){
      tErr[1] = rc[4] - cc[1];   // y cor (moving in x)
      rc[3] = cc[0];
    }
    else{
      tErr[1] = rc[1] - cc[1];  // y cor (moving in y)
      rc[4] = cc[1];
      rst = 1;
    }
  }

  if (xyz == 1){
    tErr[1] = rc[1] - cc[1];  // y cor
    if (abs(tErr[1]) > buffer[1] && rst==0){
      tErr[0] = rc[3] - cc[0];   // x cor (moving in y)
      rc[4] = cc[1];
    }
    else{
      tErr[0] = rc[1] - cc[0];  // x cor (moving in x)
      rc[3] = cc[0];
      rst = 1;
    }
  }

  if (xyz == 2){
    tErr[0] = rc[0] - cc[0];
    if (abs(pointOnRamp - cc[1]) > buffer[1] && rst == 0){
      tErr[1] = pointOnRamp - cc[1];  // y cor 1st(moving in y)
      tErr[0] = rc[3] - cc[0];
      rc[4] = cc[1];
    }
    else if (abs(tErr[0]) > buffer[0] && (rst == 0 || rst == 1) ){
      tErr[1] = rc[4] - cc[1];  // y cor 1st(moving in x)
      rc[3] = cc[0];
      rst = 1;
    }
    else{
      tErr[1] = rc[1] - cc[1];  // y cor 2nd(moving in y)
      rc[4] = cc[1];
      rst = 2;
    }
  }
}

int RequiredCordsChanged(){
  int j;
  int change = 0;
  int bufr = ((buffer[0]+buffer[1])/2) + 2;     // can create problem if next required co-ordinates are too close like in bufr of 4.
  for(j=0;j<3;j++){
    if( abs(rcp[j] - rc[j]) > bufr){
      change++;
    }
  }
  if(firstTimeChange == 0){
  change = 1;
  firstTimeChange = 1;
  }
  return change;
}


int move() {
  // oldTime = micros();
  target_cords(xyz);
  if(RequiredCordsChanged()){

    reset_pid();
    float Xcord = abs(rc[0] - cc[0]);
    float Ycord = abs(rc[1] - cc[1]);
    if (Xcord>=Ycord){
      gernyS[0] = Xcord;
      gernyS[1] = 0;
    }
    else{
      gernyS[0] = Ycord;
      gernyS[1] = 1;
    }
    
  }
  if(RequiredCordsChanged() > 1){
    rst = 0;
  }

  rcp[0] = rc[0];
  rcp[1] = rc[1];
  rcp[2] = rc[2]; 
    
  target_cords(xyz);

  if (abs(tErr[0]) < buffer[0]){
    Xreset_pid();
  }
  if (abs(tErr[1]) < buffer[1]){
    Zreset_pid();
  }
  if (abs(rotError) < buffer[2]){
    Rreset_pid();
  }
  // start moving
  if (abs(tErr[0]) > buffer[0] || abs(tErr[1]) > buffer[1]  ||  abs(rotError) > buffer[2])
  {
    newTime = micros();
    dt = (newTime - oldTime)/1000;
    oldTime = newTime;

    if (dt > 4){
      dt = 1.2;       //for avoiding acumulation time value due to calling this function after some time passed.
    }

    // pid corections
    if (abs(tErr[0]) > buffer[0]){
    xxPid(tErr[0], xPid, xCorrData[1], xCorrData[2], dt);
    }
    else{
      Xreset_pid();
    }
    if (abs(tErr[1]) > buffer[1]){
    zzPid(tErr[1], zPid, zCorrData[1], zCorrData[2], dt);
    }
    else{
      Zreset_pid();
    }
    if (abs(rotError) > buffer[2]){
    rrPid(rotError, rPid, rCorrData[1], rCorrData[2], dt);
    }
    else{
      Rreset_pid();
    }
    velocityEquations();
    
    // pid correction mapping
    // Serial.print(pwm_limit);
    pid_map(pwm_limit,vFL,vBR,vBL);
    int vFL3=pid_maped[0];
    int vBR3=pid_maped[1];
    int vBL3=pid_maped[2];
    
    // pid values to motor
    motorPwm(vFL3, vBR3, vBL3);

  // Serial.print(pwm_limit);
  // Serial.print("  ");
  // Serial.print(cc[0]);
  // Serial.print(" ");
  // Serial.print(cc[1]);
  // Serial.print(" ");
  // Serial.print(cc[2]);
  // Serial.print("  ");

  // Serial.print(rc[0]);
  // Serial.print(" ");
  // Serial.print(rc[1]);
  // Serial.print(" ");
  // Serial.print(rc[2]);
  // Serial.print("  ");

  // Serial.print(tErr[0]);
  // Serial.print(" ");
  // Serial.print(tErr[1]);
  // Serial.print(" ");
  // Serial.print(rotError);
  // Serial.print("  ");

  // Serial.print(xCorrData[0]);
  // Serial.print(" ");
  // Serial.print(zCorrData[0]);
  // Serial.print(" ");
  // Serial.print(rCorrData[0]);
  // Serial.print("  ");
    
  // Serial.print(vFL3);
  // Serial.print(" ");
  // Serial.print(vBR3);
  // Serial.print(" ");
  // Serial.print(vBL3);
  // Serial.println(" ");
    return 0;
   }
   // reset for next pid run.
  else{
    reset_pid();
    rst = 0;
    return 1;
  }
}

int paddyRead(){
  // int paddi = digitalRead(paddy);
  
  // byte one = 1;
  // Transmit(SlaveAdr,one);
  // int paddi = Request(SlaveAdr,1);
  int paddi = 0;
  if(cc[1] < -150)
  {
    paddi = 1;
  }
  // if(window>1000){
  //   paddi = 1;
  // }

  return paddi;
  // return 0;
}

int count;
int silo;

int ballPutInSilo(byte siloState){
  // int silo = digitalRead(sillo);

  // Transmit(SlaveAdr,siloState<<1);
  // int silo = Request(SlaveAdr,1);
  silo = 0;

  if(siloState==2){
    silo = 1;
  }
  if(count>2){
    silo = 1;
    count = 0;
  } 
  return silo;
  // return 0;
}

// int cnt=0;
// int navigator = 0; // take dicision of silo navigation or storage ball navigation and renavigate from returning to viwepoint to detect again and go. 
// float ptime = 0;
// float pTime = 0;
// float window = 0;
// int winfirst = 0;
// int silo_no = 0;
// int data = 0;
// int data2 = 0;

void gradualpwm(int gerny){
  if(gerny < 50){
    if(pwm_limit<higherPWM){
      pwm_limit++;
      dec = gerny;
    }
  }
  if(gerny >= 50){
    if(pwm_limit>lowerPWM && gerny >= 100-dec-beforeTimeBias){
      pwm_limit--;
    }
  }

  if(gerny >= 100){
    pwm_limit=lowerPWM;
  }
}

#define Xup 8
#define Xdn 9
#define Yup 10
#define Ydn 11
#define mult 12

int mul = 1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // Wire.begin();
  Wire1.begin();
  IMU.begin();
  IMU.setExtCrystalUse(true);
  pinMode(f1, OUTPUT);
  pinMode(b1, OUTPUT);
  pinMode(b2, OUTPUT);
  pinMode(f1d, OUTPUT);
  pinMode(b1d, OUTPUT);
  pinMode(b2d, OUTPUT);
  pinMode(b2d, OUTPUT);
  pinMode(13, OUTPUT);
  // pinMode(Xup, INPUT);
  // pinMode(Xdn, INPUT);
  // pinMode(Yup, INPUT);
  // pinMode(Ydn, INPUT);
  // pinMode(mult, INPUT);

   if (!ads.begin()) {
    // Serial.println("Failed to initialize ADS.");
    while (1);
  }  
  while(timee < 5000){
    timee = millis();
  }
  // digitalWrite(13,HIGH);
//  if(digitalRead(retryPin)==1){
//    reX = -548.5;
//    reY = -20;
//  }
}

void loop() {

  Wire1.requestFrom(8, 1);

  if(Wire1.available()){
    Mechanismdata = Wire1.read();
  }

  if((cc[0] <= -55) && (sortFlagLeft == 1)){
    sortFlagLeft = 0;
    Wire1.beginTransmission(8);
    Wire1.write(4);
    Wire1.endTransmission();
    sortFlagRight = 1;
  }

  else if((cc[0] > -55) && (sortFlagRight == 1)){
    sortFlagRight = 0;
    Wire1.beginTransmission(8);
    Wire1.write(5);
    Wire1.endTransmission();
    sortFlagLeft = 1;
  }

  

  // co-ordinate updates
  pose();


  // receiveDataBall();
  // receiveDataSilo();
  if(navigator == 1){
    receiveDataBall();
  }
  if(navigator == 2){
    receiveDataSilo();
  }
  rotErrorRange();

  // Serial.print("navigator: ");
  // Serial.print(navigator);


  // if (digitalRead(mult)==1){
  //   mul=40;
  // }
  // else{
  //   mul = 1;
  // }
  // if (digitalRead(Xup)==1){
  //   reX = reX + 0.01*mul;
  // }
  // if (digitalRead(Xdn)==1){
  //   reX = reX - 0.01*mul;
  // }
  // if (digitalRead(Yup)==1){
  //   reY = reY + 0.01*mul;
  // }
  // if (digitalRead(Ydn)==1){
  //   reY = reY - 0.01*mul;
  // }
//  reX = reX - 0.2;

  // data2 = data2 + data;
  // // Serial.println(data2);

  // // move to area 3
  // if ( data2 == 0){
  //   rc[0] = -560*scale;
  //   rc[1] = 0*scale;
  // }





  //''''''''''''''''''''''''

  if(window>10){
    motorPwm(0, 0, 0);
  }
  
  // actual motion
  data = move();

  
  // window in ms to cheak for how much time the bot is stuck in same place.
  if(data == 0){
    winfirst = 0;
     window = 0;
     cnt = 0;
  }
  if(data == 1){
    if(data == 1 && winfirst == 0){
        ptime = millis();
    }
    window = millis() - ptime;
    winfirst = 1;
  }
  // window end

  gerny = abs( gernyS[0] ) - abs( tErr[(int)gernyS[1]] );
  gerny = map(gerny,0,abs(gernyS[0] - shortPid), 0,100);
  // Serial.print("win :");
  // Serial.print(window);
  // Serial.println("gr0:");
  // Serial.println(gernyS[0]);
  // Serial.println("gr1:");
  // Serial.println(gernyS[1]);
  // Serial.println(" :");
  // Serial.print("ger :");
  // Serial.print(gerny);
  // Serial.print(" :");
  if(millis()-pTime > stepIncrement){
    gradualpwm(gerny);
    pTime = millis();
  }

//  data2 = data2 + data;
//  Serial.println(data2);

 
  // start ball detection
  if (data2 == 0){
    Serial.print(depthCamera);
    navigator = 1;
    data2 = 1;
    // cnt=4;
  }
  ///////////
//////////////////////////////////// comment /////////////////////

  if (navigator == 1){
  //  if(paddyRead() == 1 || window > 500){
  // receiveDataBall();
  if(Mechanismdata == 1){
    Serial.print(siloCamera);
    rc[0] = cc[0];
    rc[1] = cc[1];
    // xyz = 1;
    navigator = 2;
  }
  if(Mechanismdata == 2){
    Serial.print(depthCamera);
    rc[0] = BallViwePoint[0];
    rc[1] = BallViwePoint[1];
    // xyz = 1;
    // navigator = 2;
  }
    // if (window>3000){
    //   Serial.print(depthCamera);
    //   rc[0] = BallViwePoint[0];
    //   rc[1] = BallViwePoint[1];

    //   // xyz = 2;
    // }

  }

/////////////////////////////////////////////////////////////////////////////////////////////////
  if(Mechanismdata!=0){
  Serial.print(" silo_no: ");
  Serial.print(silo_no);
  Serial.print(" Mechanismdata: ");
  Serial.println(Mechanismdata);
  }

  if (navigator == 2 && siloData == 1 && data == 1){
    // rcs[5] = predefine cords           silo_preference[3] ,silo_status[3]  = from python
    if(silo_no == 0 && cntSilo_no == 0){
      sendstate=1;
      cntSilo_no=1;
    }
    if(silo_no!=0){
      cntSilo_no=0;
    }

    if(Mechanismdata == 3){
      sendstate=0;
      Serial.print(depthCamera);
      rc[0] = BallViwePoint[0];
      rc[1] = BallViwePoint[1];

      // timee = millis();
      // while(Delay < 4000){
      //   Delay = millis() - timee;
      // }
      navigator = 1; 
      silo_no = 0;
      siloData = 0;
      }

    if((Mechanismdata == 4 && silo_no<=2)){     //if((Mechanismdata == 4 && silo_no<=2) && navigator == 2)
      
        silo_no = silo_no + 1;
        Serial.print("siloapnd:");
        Serial.println(silo_no);

        if(silo_no<=2){
          sendstate=1;
        }
    }

    if ((silo_no<3) && navigator == 2){
      Serial.print("rcupdate");
    rc[0] = rcs[silo_preference[silo_no]][0];
    rc[1] = rcs[silo_preference[silo_no]][1];
    Serial.println(rc[0]);
    Serial.print(" ");
    Serial.println(rc[1]);
    // xyz = 1;

    if(sendstate==1  && (abs(cc[0] - rc[0]) <= 2) && (abs(cc[1] - rc[1]) <= 2) ){
      Serial.print("state send:");
      Serial.print(silo_status[silo_no]);
    // digitalWrite(13,HIGH);
    Wire1.beginTransmission(8);
    Wire1.write(silo_status[silo_no]);
    Wire1.endTransmission();
    // delay(100);
    // digitalWrite(13,LOW);
    // Serial.print(" silo state : ");
    // Serial.print(silo_status[silo_no]);
    sendstate=0;
    }

    }

    

    if ((silo_no>=3) && navigator == 2){
      rc[0] = SiloViwePoint[0];
      rc[1] = SiloViwePoint[1];
      // xyz = 0;
      if(abs(cc[0] - SiloViwePoint[0]) <= 2 && abs(cc[1] - SiloViwePoint[1]) <= 2)
      {
      silo_no = 0;
      siloData = 0;
      Serial.print(siloCamera);
      // count++;
      }

    }


  }
  // Serial.println(" ");

}

  /*
  if (navigator == 1){  [storage zone navigation](navigator = 1;)
      CLT = { center navigation (in front of plus after ramp)(L trass x first) till camera detects ball.
        move to ball imidiatly in between center L tras if rc updates by recive data.  }
      if stuck in same place (window >10s) not paddy rice in then navigate back to viwepoint(again recives rc in between and move to ball)
      if not stuck then paddy rice in (paddyRead() = 1) (rc[] = cc[]) wait there.(navigator = 2;)
  }

  if (navigator == 2){  [silo zone navigation]
    $ wait until  reciveSiloData() updates.
    navigate to silo by reciveSiloData() cordinates , L trace (y first).
    @ stop and send 1st silo status to mechanisum arduino in 2 bits.
    if siloputornot() == 1 then put ball in silo and (navigator = 1;)start CLT 

    if siloputornot() == 0 then move to next silo @ and now repeat @-----@ until (navigator = 1;) 

    if navigator != 1 at 3rd silo then move to viwepoit and (Serial.print(siloCamera);) $ now $-----$ will be repeat.

  }
  */

// }




