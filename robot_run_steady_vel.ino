
int botvel;
volatile int counterL_f = 0;
volatile int counterR_f = 0;

volatile int counterL_b = 0;
volatile int counterR_b = 0;

int dir1 = 38, pwm1 = 44, dir2 = 39,  pwm2 = 45;


int ok = 0;
    
void countL_forward() {      
         counterL_f++;   
            
}

void countR_forward() {      
         counterR_f++; 
}

void countL_backward() {      
         counterL_b++;  
}

void countR_backward() {      
         counterR_b++; 
}

void right(int vel) {
  

   analogWrite(pwm1, vel); 
   analogWrite(pwm2, vel); 
    
   digitalWrite(dir1,0);    //0
   digitalWrite(dir2,1);    //1

}


void left(int vel) {
  

    analogWrite(pwm1, vel); 
    analogWrite(pwm2, vel); 
    
   digitalWrite(dir1,1);
   digitalWrite(dir2,0);

}

void backward() { 

    
   digitalWrite(dir1,1);
   digitalWrite(dir2,1);

}

void forward() {      
    
   digitalWrite(dir1,0);
   digitalWrite(dir2,0);

}

void stop() {             //       pwm1 = 13; dir1 = 12; pwm2 = 11;  dir2 = 10;
  
  analogWrite(pwm1, 0); 
  analogWrite(pwm2, 0); 
}

void setup()
{
 //       dir1 = 22; pwm1 = 44; dir2 = 23;  pwm2 = 45;
    pinMode(dir1, OUTPUT); 
    pinMode(pwm1, OUTPUT); 
    pinMode(dir2, OUTPUT); 
    pinMode(pwm2, OUTPUT); 
   
    
    attachInterrupt(digitalPinToInterrupt(2), countL_forward, RISING); //interrupt pins for encoder Achange
    attachInterrupt(digitalPinToInterrupt(3), countR_forward, RISING);
    attachInterrupt(digitalPinToInterrupt(18), countL_backward, RISING); //interrupt pins for encoder Achange
    attachInterrupt(digitalPinToInterrupt(19), countR_backward, RISING);



    
   Serial.begin(9600);
}

void slow() {
  int i;
  
  for(i=128;i>0;i-=10) {
    analogWrite(pwm1,i);
   analogWrite(pwm2,i);
  }
}

float get_vel() {
 int val1,val2;
 
   val1 = counterL_f;   
   delay(10);
   val2 = counterL_f;   
    
   return  ( abs(val1-val2) * 0.000971905 ) / 0.01 ;
}

float get_vel_back() {
 int val1,val2;
 
   val1 = counterL_b;   
   delay(10);
   val2 = counterL_b;   
    
   return  ( abs(val1-val2) * 0.000971905 ) / 0.01 ;
}
//left
//210 count / round , wheel r = 3.25 cm
// 2PiR = 0.2041
//so 1 pulse = 0.000971905 m.

//right
//207 count / round , wheel r = 3.25 cm
// 2PiR = 0.2041
//so 1 pulse = 0.000157005 m.

void loop() {
int i;
float v;
   
      forward();
      i = 10;
      while(1) {
        analogWrite(pwm1, i); 
        analogWrite(pwm2, i + 7); 
        if(get_vel() >= 0.2)
           break;
        i += 10;
      }
      
         
     delay(3000);    
     stop();
     delay(1000); 
     
      i = 10;
      while(1) {
        analogWrite(pwm1, i); 
        analogWrite(pwm2, i + 7); 
        if(get_vel() >= 0.3)
           break;
        i += 10;
      }
      
         
     delay(3000);    
    
     stop();
     delay(1000); 
//-----------------------------------------
     backward();
      i = 10;
      while(1) {
        analogWrite(pwm1, i + 7); 
        analogWrite(pwm2, i); 
        if(get_vel_back() >= 0.2)
           break;
        i += 10;
      }
      
   
     delay(3000);    
     stop();
     delay(1000);   

      i = 10;
      while(1) {
        analogWrite(pwm1, i + 7); 
        analogWrite(pwm2, i); 
        if(get_vel_back() >= 0.3)
           break;
        i += 10;
      }

     delay(3000);    
     stop();
     delay(1000); 

}

