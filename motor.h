#ifndef MOTOR_H
#define MOTOR_H
#include <stdlib.h>
#include <signal.h>
#include <wiringPi.h>
#include <softPwm.h>

enum Direction{
	FORWARD,
	BACKWARD
};

class Motor{
  public:
    Motor(int i_ena, int i_in1, int i_in2, int i_encA, int i_encB, float i_stepsPerRev);
    void setDirection(bool dir);
    void runOpen(int speed, bool dir);
    void runClosed(double speed, bool dir);
    void disconnect();
    int ena;
    int in1;
    int in2;
    int encA;
    int encB;
    float stepsPerRev;
    volatile long encoderPos;
    bool softPWM;
};

#endif
