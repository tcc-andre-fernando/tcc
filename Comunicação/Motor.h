/*
  Motor.h - Biblioteca para funções relacionadas ao motor.
*/
#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

class Motor
{
  public:
    Motor(int IN1, int IN2, int PWM1);
    void go(int pwm);
    

  private:
    int  _PWM1;
    int  _IN1;
    int  _IN2;

};

#endif
