#include "m3508_base.hpp"
#include "MotorPid.hpp"
#include "Timer.hpp"
#include "can.hpp"
#include "rclcpp/rclcpp.hpp"

MYTIMER m3508_timer;

// 初始化电机ID，主机ID，以及can通道
motor_3508::motor_3508(int MOTOR_ID, INT device, int CAN_NUM)
{
    motor_id = MOTOR_ID;
    VCI_CAN_DEVICE = device;
    VCI_USBCAN_NUM = CAN_NUM;
    m3508_input.id = motor_id;
}

void motor_3508::motor_3508_init(std::string Send, std::string Recv, std::string Node_name)
{
    node = rclcpp::Node::make_shared(Node_name);
    pub_m3508_input = node->create_publisher<r1_upper::msg::M3508send>(Send, 50);
    sub_m3508_feedback = node->create_subscription<r1_upper::msg::M3508rec>(Recv, 50, std::bind(&motor_3508::sub_m3508_feedback_callback, this, std::placeholders::_1));
}

void motor_3508::sub_m3508_feedback_callback(const r1_upper::msg::M3508rec::SharedPtr msg)
{
    if (msg->id == motor_id)
    {
        LastPos = CurrentPos;
        LastSpeed = CurrentSpeed;
        LastCurrent = CurrentCurrent;
        LastTorque = CurrentTorque;

        CurrentPos = msg->pos;
        CurrentSpeed = msg->speed;
        CurrentCurrent = msg->current;
        CurrentTorque = msg->current;
        // std::cout << "motor_id" << motor_id << "    CurrentCurrent:" << CurrentCurrent << std::endl;

        // std::cout << "motor_id" << motor_id << "   CurrentPos:" << CurrentPos << "    CurrentSpeed:" << CurrentSpeed << "    CurrentCurrent:" << CurrentCurrent << std::endl;
    }
}

void motor_3508::set_pos(float pos,float kp,float ki,float kd,float limit_output,float pre_current)
{
    m3508_input.pos = pos;
    m3508_input.kp = kp;
    m3508_input.kd = kd;
    m3508_input.ki = ki;
    m3508_input.limit_output = limit_output;
    m3508_input.current = pre_current;
    m3508_input.mode = M3508_POS_MODE;
    pub_m3508_input->publish(m3508_input);
}

void motor_3508::set_speed(float speed)
{
    m3508_input.speed = speed;
    m3508_input.mode = M3508_SPEED_MODE;
    pub_m3508_input->publish(m3508_input);
}

void motor_3508::set_current(float current)
{
    m3508_input.current = current;
    m3508_input.mode = M3508_CURRENT_MODE;
    pub_m3508_input->publish(m3508_input);
}

float motor_3508::Get_RM3508_Distance(MotorTypeDef motor_pos)
{
    int Distance = motor_pos.PosNow - motor_pos.PosPre;
    if (abs(Distance) > 4000)
        Distance = Distance - Distance / abs(Distance) * 8192;
    return ((float)Distance * 360.0f / 3591.0f * 187.0f / 8192.0f) * M_PI / 180;
}