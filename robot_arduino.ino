#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

int OdomCount = 0;

int OdomWait = 3;
static const float GYRO_SENS  =  160;  

int LMotor = 1;
int RMotor = 0;

bool left_forward;
bool right_forward;

int MotorNum[2] = {LMotor, RMotor};

double Vels[2] = {0,0};
int CVEL[2]= {0,0};
int Mspeeds[2] = {0,0};


double WCS[2] = {0,0};
long EncoderVal[2] = {0,0};
double DDis[2] = {0,0};
long Time[2] = {0,0};

float WheelSeparation = 0.24; //0.17
float WheelDiameter = 0.07;  //60 to fast
int TPR = 700; //Encoder ticks per rotation  

volatile int counterL = 0;
volatile int counterR = 0;

int AccParam = 3; //acceleration multiplier.

int dir;

int bot_vel;
int count = 0;

int dir1 = 38, pwm1 = 39, dir2 = 40,  pwm2 = 41;

int motor_speed_l,motor_speed_r;

ros::NodeHandle  nh;

geometry_msgs::Twist odom_msg;
ros::Publisher Pub ("ard_odom", &odom_msg);  


void countL() {
      if(WCS[0]  > 0)
         counterL++; 
      else             
         counterL--;   
}

void countR() {
      if(WCS[1]  > 0)
         counterR++; 
      else            
         counterR--;   
}

void forward() {    
   digitalWrite(dir1,0);
   digitalWrite(dir2,1);
   dir = 0;
}


void backward() {
   digitalWrite(dir1,1);
   digitalWrite(dir2,0);
   dir = 1;
}

void left() {    
   digitalWrite(dir1,0);
   digitalWrite(dir2,0);
   dir = 2;
}

void right() {    
   digitalWrite(dir1,1);
   digitalWrite(dir2,1);
   dir = 3;
}

void stop() {             
   analogWrite(pwm1, 0); 
   analogWrite(pwm2, 0); 
}

void velCallback( const geometry_msgs::Twist& CVel){ 
double vel_x = CVel.linear.x;
double vel_th = CVel.angular.z;
double right_vel = 0.0;
double left_vel = 0.0;

      // turning
      if(vel_x == 0){ 
          right_vel = vel_th * WheelSeparation / 2.0;
          left_vel = (-1) * right_vel;
      }
      // forward / backward
      else if(vel_th == 0){ 
          left_vel = right_vel = vel_x;
      }
      // moving doing arcs
      else{ 
          left_vel = vel_x - vel_th * WheelSeparation / 2.0;
          right_vel = vel_x + vel_th * WheelSeparation / 2.0;           
      }
      //write new command speeds to global vars 
      WCS[0] = left_vel;
      WCS[1] = right_vel;     
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel" , &velCallback);

//motor write speed - in motor units
double MWS[2]= {0,0};

double CorrectedSpeed(int M, double CVel){
  //if fist time in program return 0 and init time vars
    if(Time[0]==0 && Time[1] == 0){
      Time[0] = millis();
      Time[1] = millis();
      return 0;
    }

//read encoder ticks
    if(M == 0){
      EncoderVal[0] = counterL;
      counterL = 0;
    }
    if(M == 1){
      EncoderVal[1] = counterR;
      counterR = 0;
    }

  //differencial of time in seconds
  long T = millis();
  int DTime = T-Time[M];
  Time[M] = T;
  
  //diferential of distance in meters
  DDis[M] = TicksToMeters(EncoderVal[M]);
  
  //calculate short term measured velocity
  double EVel = (DDis[M]/DTime)*1000;

  //save to publish to /ard_odom
  Vels[M] = EVel;

  EVel = abs(EVel);
  CVel = abs(CVel);
  
  double dif = EVel - CVel;  
  
  MWS[M] = MWS[M]-(dif * (AccParam));

  if(MWS[M]<0)
    MWS[M] = 0;

  if(CVel == 0) 
    MWS[M] = 0;

  //DEBUG
  CVEL[M] = MWS[M];
  
  return MWS[M];

}

double TicksToMeters(int Ticks){
return (Ticks*3.14*WheelDiameter)/TPR;
}

//motor codes///////////////////////////////////////////////////////////
void MotorWrite(){
int DIR;
int min_speed;

    for(int i = 0; i<2; i++){  
        //correct speed with encoder data
        double MSpeed = CorrectedSpeed(i, WCS[i]);        
        
       if(i == 0)          
            motor_speed_l = MSpeed;
        else if(i == 1)            
            motor_speed_r = MSpeed;
    }
    motorGo(motor_speed_l,motor_speed_r);
}

void motorGo(uint8_t l, uint8_t r)
{
    
    if(WCS[0] > 0 && WCS[1] > 0) {
      forward();      
      if(WCS[0] == WCS[1]) {
        analogWrite(pwm1,l);
        analogWrite(pwm2,l);
      }
      else {
        analogWrite(pwm1,l);
        analogWrite(pwm2,r);
      }
        
    }else if(WCS[0] < 0 && WCS[1] < 0) {
      backward();      
      if(WCS[0] == WCS[1]) {
        analogWrite(pwm1,l);
        analogWrite(pwm2,l);
      }
      else {
        analogWrite(pwm1,l);
        analogWrite(pwm2,r);
      }
    }
    else if(WCS[0] > 0 && WCS[1] < 0) {
      left();
      analogWrite(pwm1,l);
      analogWrite(pwm2,l);
    }
    else if(WCS[0] < 0 && WCS[1] > 0) {
      right();
      analogWrite(pwm1,l);
      analogWrite(pwm2,l);
    }
    else{
        analogWrite(pwm1,l);
        analogWrite(pwm2,r);
    }
 }


void setup() {  
  
  nh.initNode();  
  nh.advertise(Pub);  
  nh.subscribe(sub);

    pinMode(38, OUTPUT); 
    pinMode(39, OUTPUT); 
    pinMode(40, OUTPUT); 
    pinMode(41, OUTPUT); 

    pinMode(2, INPUT_PULLUP);
    pinMode(3, INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(2), countL, RISING); //interrupt pins for encoder Achange
    attachInterrupt(digitalPinToInterrupt(3), countR, RISING);


  Wire.begin(); 
 
  // ==================== MPU6050 ============================
  accelgyro.initialize();  

  //Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // Starts up with accel +/- 2 g and gyro +/- 250 deg/s scale
  accelgyro.setI2CBypassEnabled(true); // set bypass mode
}

void loop() {
 int imu;
 nh.spinOnce();
   
    if(OdomCount > OdomWait){
          odom_msg.linear.x = Vels[0];
          odom_msg.linear.y = Vels[1];
          accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  
         
          
          if(Vels[0] == 0 and Vels[1] == 0) {
               odom_msg.linear.z = 0;
          }
          else {
            imu = (int)(gz/GYRO_SENS) + 1;
            if(imu > 0)   //only my calibration
                imu += 10;    
            else if(imu < 0)
                imu -= 10;   
                
            odom_msg.linear.z = imu;            
          }                      
        Pub.publish(&odom_msg);
    
    }
    else
        OdomCount++;

    MotorWrite(); //Takes WCS and corrects speed of motors with encoders 
     
}
