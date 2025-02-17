#ifndef M3508_BASE_HPP
#define M3508_BASE_HPP

#include "Motor/Cybergear/controlcan.h"
#include <iostream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "r1_upper/msg/m3508send.hpp"
#include "r1_upper/msg/m3508rec.hpp"

#define M3508_POS_MODE 1
#define M3508_SPEED_MODE 2
#define M3508_CURRENT_MODE 3

#define CURRENT_TO_TORQUE 0.000366211 // 20 * 16384 * 0.3

typedef struct
{
    float PositionExpected;
    float PositionMeasure;
    float SpeedExpected;
    float SpeedMeasure; //
    float CurrentExpected;
    float CurrentMeasure; //

    short PosPre;
    short PosNow;

    int32_t PWM;

} MotorTypeDef;

class motor_3508
{
private:
    int reclen = 0;


public:
    int VCI_USBCAN_NUM;
    int VCI_CAN_DEVICE;
    int motor_id = 0;
    float CurrentPos = 0;
    float CurrentSpeed = 0;
    float CurrentCurrent = 0;
    float CurrentTorque = 0;
    float LastPos = 0;
    float LastSpeed = 0;
    float LastCurrent = 0;
    float LastTorque = 0;
    float PositionExpected = 0;
    float SpeedExpected = 0;
    float CurrentExpected = 0;
    double M3508_REDUCE_RATIO = 3591.0 / 187.0;
    int32_t PWM;

    MotorTypeDef motortypedef;
    VCI_CAN_OBJ CmdSend[1];
    VCI_CAN_OBJ DataRecv[2500];

    r1_upper::msg::M3508send m3508_input;
    r1_upper::msg::M3508rec m3508_feedback;
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<r1_upper::msg::M3508send>::SharedPtr pub_m3508_input;
    rclcpp::Subscription<r1_upper::msg::M3508rec>::SharedPtr sub_m3508_feedback;

    motor_3508(int MOTOR_ID, INT device, int CAN_NUM);
    void motor_3508_init(std::string Send, std::string Recv, std::string Node_name);
    void set_pos(float pos,float kp,float ki,float kd,float limit_output,float pre_current);
    void set_speed(float speed);
    void set_current(float current);
    float Get_RM3508_Distance(MotorTypeDef motor_pos);
    void sub_m3508_feedback_callback(const r1_upper::msg::M3508rec::SharedPtr msg);

};

// static void sub_m3508_feedback_callback(const r1_upper::msg::M3508rec::SharedPtr msg);

#endif