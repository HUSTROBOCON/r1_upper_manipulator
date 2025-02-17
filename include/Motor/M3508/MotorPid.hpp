#ifndef __MOTOR_PID_HPP__
#define __MOTOR_PID_HPP__

#include <iostream>
#include <cmath>

typedef struct{
    float Kp;
    float Ki;
    float Kd;
    float LimitOutput;
    float LimitIntegral;
    float Integral;
    float PreError;

    int SectionFlag;
    float KpMax;
    float ErrorLine;
}ClassicPidStructTypedef;

extern ClassicPidStructTypedef MotorPositionPid1, MotorSpeedPid1, MotorCurrentPid1;
extern ClassicPidStructTypedef MotorPositionPid2, MotorSpeedPid2, MotorCurrentPid2;

void MotorCurrentPidInit(void);
void MotorSpeedPidInit(void);
void MotorPositionPidInit(void);
void MotorPidInit(void);

float ClassicPidRegulate(float Reference, float PresentFeedback, ClassicPidStructTypedef* PidStruct);

#endif
