#ifndef __GOBASE__
#define __GOBASE__

#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

#define UPPER_ARM_PORT "/dev/upper_arm" 
#define UNDER_ARM_PORT "/dev/under_arm" 
#define WRIST_PORT "/dev/wrist" 
#define RECEIVE_BALL_PORT "/dev/receive_ball" 

#define GO1_POSITION 0.6f              // GO1电机初始位置 vel 0.3   0.3f
#define GO2_POSITION -0.1f              // GO2电机初始位置
#define GO3_POSITION 0.3f
#define DRIBBLE_PRE_GO1_POSITION 0.4f  // GO2电机初始位置0.4
#define DRIBBLE_PRE_GO2_POSITION -0.65f // GO2电机初始位置-0.85 //12_30  -0.52
#define DRIBBLE_PRE_GO3_POSITION 1.82f // GO2电机初始位置-0.85  //12_30  1.55
#define DRIBBLE_GO3_POSITION 2.5f // GO2电机初始位置-0.85  //12_30  2.825

// #define DRIBBLE_PRE_GO1_POSITION 2.2f  // GO2电机初始位置0.4
// #define DRIBBLE_PRE_GO2_POSITION -2.4f // GO2电机初始位置-0.85 //12_30  -0.52
// #define DRIBBLE_PRE_GO3_POSITION 0.0f // GO2电机初始位置-0.85  //12_30
// #define DRIBBLE_GO3_POSITION 2.4f // GO2电机初始位置-0.85  //12_30  2.825

class GO_Drive
{
public:
    MotorCmd cmd;
    MotorData data;
    int ID;
    float Zero_pos;
    float Pos_now;
    int GO_flag = 0;    // go到位标志
    int delay_flag = 0; // 小米延迟判断标志
    SerialPort *serial;
    void GO_init(int id, std::string temp_serial_usb);
    void GO_Velocity_Time(float speed, int time);
    void GO_Velocity(float speed, float kd);
    void GO_Position(float pos, float kp,float kd);
    void GO_Dumping(float kd);
    void GO_moment(float tau);
    void GO_Stop();
    void GO_Movement(float kp, float kd, float q, float dq, float tau);
    void GO_position_speed(float GO_kp, float GO_kd, float GO_q, float GO_dq, float delay_flag_q);
    void GO_shoot(float GO_kp, float GO_kd, float GO_q, float GO_dq, float delay_flag_q, int mode=0);
};

#endif