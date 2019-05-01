#include "motor.h"

Motor::
Motor(int i_ena, int i_in1, int i_in2, int i_encA, int i_encB, float i_stepsPerRev)
  :ena(i_ena), in1(i_in1), in2(i_in2), encA(i_encA), encB(i_encB), stepsPerRev(i_stepsPerRev)
{
  wiringPiSetup();
  if(ena == 1){
    //hardware PWM
    softPWM = false;
    pinMode(ena,PWM_OUTPUT);
    pwmWrite(ena,0);
  }else{
    //software PWM
    softPWM = true;
    softPwmCreate(ena,0,100);
  }
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(encA,INPUT);
  pinMode(encB,INPUT);
}

void
Motor::
setDirection(bool dir)
{
  if(softPWM)
    softPwmWrite(ena,0);
  else	
    pwmWrite(ena,0);
  if(!dir){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
}

void
Motor::
runOpen(int speed, bool dir)
{
  setDirection(dir);
  if(speed<0)
    speed = 0;
  if(speed>1023)
    speed = 1023;
  if(softPWM)
    softPwmWrite(ena,speed);
  else
    pwmWrite(ena,speed);
}

void
Motor::
runClosed(double speed, bool dir)
{
  setDirection(dir);
  if(speed<0)
    speed = 0;
  if(speed>1023)
    speed = 1023;
  if(softPWM)
    softPwmWrite(ena,speed);
  else
    pwmWrite(ena,speed);
}

void
Motor::
disconnect()
{
  runOpen(0,FORWARD);
  digitalWrite(in1,LOW);
  digitalWrite(in2,LOW);
}
