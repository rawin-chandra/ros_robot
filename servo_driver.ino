/*
Edit by T.Rawin Chaisittiporn (T. S)
for use with servo motor driver and arduino 
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

 #define MIN_PULSE_WIDTH       650
#define MAX_PULSE_WIDTH       2350
#define DEFAULT_PULSE_WIDTH   1500
#define FREQUENCY             50

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
 
void setup() {
  Serial.begin(9600);
  Serial.println("16 channel PWM test!");
  pwm.begin(); 
  pwm.setPWMFreq(FREQUENCY);  // This is the maximum PWM frequency  1600
 
  // save I2C bitrate
  //uint8_t twbrbackup = TWBR;
  // must be changed after calling Wire.begin() (inside pwm.begin())
  //TWBR = 12; // upgrade to 400KHz!
 
}

int pulseWidth(int angle)
{
  int pulse_wide, analog_value;
  pulse_wide   = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  //Serial.println(analog_value);
  return analog_value;
}

void loop() {
  Serial.println("1 140");
  delay(1000);
  pwm.setPWM(1, 0, pulseWidth(140));  
  delay(2000);

  Serial.println("1 135");
  delay(1000);
  pwm.setPWM(1, 0, pulseWidth(135));  
  delay(2000);

  
  Serial.println("2 80");
  delay(1000);
  pwm.setPWM(2, 0, pulseWidth(80));  
  delay(2000);

  Serial.println("2 90");
  delay(1000);
  pwm.setPWM(2, 0, pulseWidth(90));  
  delay(2000);

  
 Serial.println("3 120");
  delay(1000);
  pwm.setPWM(3, 0, pulseWidth(120));  
  delay(2000);

  Serial.println("3 90");
  delay(1000);
  pwm.setPWM(3, 0, pulseWidth(90));  
  delay(2000);

 Serial.println("4 170");
  delay(1000);
  pwm.setPWM(4, 0, pulseWidth(170));  
  delay(2000);

  Serial.println("4 180");
  delay(1000);
  pwm.setPWM(4, 0, pulseWidth(180));  
  delay(2000);

  Serial.println("6 0");
  delay(1000);
  pwm.setPWM(6, 0, pulseWidth(0));  
  delay(2000);

  Serial.println("6 90");
  delay(1000);
  pwm.setPWM(6, 0, pulseWidth(90));  
  delay(2000);
    
}
