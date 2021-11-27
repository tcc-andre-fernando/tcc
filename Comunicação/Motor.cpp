#include "Arduino.h"
#include "Motor.h"

Motor::Motor(int IN1, int IN2, int PWM1)
{
  _IN1 = IN1;
  _IN2 = IN2;
  _PWM1 = PWM1;            
  
  pinMode(_IN1, OUTPUT);
  pinMode(_IN2, OUTPUT);
  pinMode(_PWM1, OUTPUT);
}

void Motor::go(int pwm){
  
  if (pwm > 250){pwm =  250;}
  if (pwm < -250){pwm = -250;}

  //Serial.print(pwm);

  if (pwm<0){
    analogWrite(_PWM1, -pwm);
    digitalWrite(_IN1, LOW);
    digitalWrite(_IN2, HIGH);
    }
  if (pwm>0){
    analogWrite(_PWM1, pwm);
    digitalWrite(_IN1, HIGH);
    digitalWrite(_IN2, LOW);  
    }
  if (pwm == 0){
    analogWrite(_PWM1, 0);
    digitalWrite(_IN1, LOW);
    digitalWrite(_IN2, LOW);
    }
  return;
}
