#ifndef FRAME_COPY_HPP
#define FRAME_COPY_HPP

#include "Motor/Cybergear/controlcan.h"
#include <iostream>
#include <string>
#include <cmath>
#include "r1_upper/msg/cybergearsend.hpp"
#include "r1_upper/msg/cybergearrec.hpp"
#include "rclcpp/rclcpp.hpp"
// #include <std_msgs/msg/string.hpp>

#define CYBERGEAR_SET_ZERO_MODE 1
#define CYBERGEAR_ENABLE_MODE 2
#define CYBERGEAR_POSITION_MODE 3
#define CYBERGEAR_VELOCITY_MODE 4
#define CYBERGEAR_OPERATION_CONTROL_MODE 5

#define can_master_id 0xfe
#define motor_1_id 0x01
#define motor_2_id 0x02
#define motor_3_id 0x03
#define motor_4_id 0x04
#define motor_5_id 0x05
#define motor_6_id 0x06
#define motor_7_id 0x07
#define motor_8_id 0x08

#define get_motor_id 0x00
#define operation_control 0x01
#define motor_feed_back_data 0x02
#define enable 0x03
#define cyber_stop 0x04
#define set_zero_position 0x06
#define set_can_id 0x07
#define read_parameter 0x11
#define write_parameter 0x12
#define fault_feed_back 0x15

#define run_mode (UINT)0x7005
#define iq_ref (UINT)0x7006
#define spd_ref (UINT)0x700A
#define imit_torque (UINT)0x700B
#define cur_kp (UINT)0x7010
#define cur_ki (UINT)0x7011
#define cur_filt_gain (UINT)0x7014
#define loc_ref (UINT)0x7016
#define limit_spd (UINT)0x7017
#define limit_cur (UINT)0x7018

#define ope_ctrl 0x00
#define cyber_pos 0x01
#define spd 0x02
// #define current      0x03

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -12.0f
#define T_MAX 12.0f

#define XM_POSITION -0.5f // 老爪-0.3f  新爪 -0.6f
#define DRIBBLE_PRE_XM_POSITION -2.3f  //-2.0.//-2.6
#define DRIBBLE_XM_POSITION -2.5f   //-2.8
#define ORIGINAL_XM_POSITION 0.0f
// 小米电机初始位置

#define motor_channel "motor_msg"
#define motor_back_channel "motor_backmsg"

struct frame_id
{
    UINT mode;
    UINT master_id;
    UINT motor_id;
    UINT res;
};

class motor_drive
{
public:
    VCI_CAN_OBJ frame[1];
    frame_id FRAME_ID;
    UINT master_id;
    UINT motor_id;
    int VCI_USBCAN_NUM;
    int VCI_CAN_DEVICE;
    int Cybergear_flag = 0; // 小米到位标志
    float Current_speed = 0;
    float Current_pos = 0;
    float Current_torque = 0;
    float Last_speed = 0;
    float Last_pos = 0;
    float Last_torque = 0;
    float test_last_pos = 0;

    r1_upper::msg::Cybergearsend cybergear_input;
    r1_upper::msg::Cybergearrec cybergear_feedback;
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<r1_upper::msg::Cybergearsend>::SharedPtr pub_cybergear_input;
    rclcpp::Subscription<r1_upper::msg::Cybergearrec>::SharedPtr sub_cybergear_feedback;

    motor_drive(int MASTER_ID, int MOTOR_ID, INT device, int CAN_NUM);
    void cybergear_init(std::string Send, std::string Recv, std::string Node_name);
    void sub_cybergear_feedback_callback(const r1_upper::msg::Cybergearrec::SharedPtr msg);
    void Get_motor_id();
    void Rec();
    void Set_zero_position();
    void Set_can_id();
    UINT float2hex(float num);
    int float_to_uint(float x, float x_min, float x_max, int bits);
    void motor_enable();
    void motor_stop();
    void motor_operation_control(float torque, float MechPosition, float speed, float kp, float kd);
    void motor_position(float limit_speed, float position);
    void motor_velocity(float speed);
    void Cybergear_position_speed(float torque, float MechPosition, float speed, float kp, float kd);

    double uint_to_float(int first_byte, int second_byte, float x_min, float x_max);
    double read_angle(int first_byte, int second_byte);
    double read_speed(int first_byte, int second_byte);
    double read_torque(int first_byte, int second_byte);
    // double read_temperature(int first_byte, int second_byte);
};

#endif
