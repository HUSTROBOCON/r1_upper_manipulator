#include "Motor/Cybergear/cybergear_base.hpp"
#include "Motor/Cybergear/controlcan.h"
#include "Timer.hpp"

MYTIMER myclock;
int reclen = 0;
VCI_CAN_OBJ rec[2500]; // 接收缓存，设为3000为佳。
int ind = 0;
int j;
float temp_speed = 0;
int temp_num = 0;

// 初始化电机ID，主机ID，以及can通道
motor_drive::motor_drive(int MASTER_ID, int MOTOR_ID, INT device, int CAN_NUM)
{
    master_id = MASTER_ID;
    motor_id = MOTOR_ID;
    VCI_CAN_DEVICE = device;
    VCI_USBCAN_NUM = CAN_NUM;
}

double motor_drive::uint_to_float(int first_byte, int second_byte, float x_min, float x_max)
{
    double span = x_max - x_min;
    double offset = x_min;
    UINT temp;
    double temp_temp;
    temp = first_byte << 8 | second_byte;
    temp_temp = ((float)temp / 65535) * span + offset;
    return temp_temp;
}

// 754进制转化
int motor_drive::float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max)
        x = x_max;
    else if (x < x_min)
        x = x_min;
    return (UINT)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

UINT motor_drive::float2hex(float num)
{
    UINT *i = (UINT *)&num;
    return *i;
}

// 反馈帧读取角度
double motor_drive::read_angle(int first_byte, int second_byte)
{
    return uint_to_float(first_byte, second_byte, P_MIN, P_MAX);
}
// 反馈帧读取角速度
double motor_drive::read_speed(int first_byte, int second_byte)
{
    return uint_to_float(first_byte, second_byte, V_MIN, V_MAX);
}
// 反馈帧读取力矩
double motor_drive::read_torque(int first_byte, int second_byte)
{
    return uint_to_float(first_byte, second_byte, T_MIN, T_MAX);
}
// 反馈帧读取温度  目前读取有问题
//  double motor_drive::read_temperature(int first_byte, int second_byte){
//      return  ((double)((first_byte<<8|second_byte)))/10;
//  }

void motor_drive::cybergear_init(std::string Send, std::string Recv, std::string Node_name)
{
    node = rclcpp::Node::make_shared(Node_name);
    pub_cybergear_input = node->create_publisher<r1_upper::msg::Cybergearsend>(Send, 50);
    sub_cybergear_feedback = node->create_subscription<r1_upper::msg::Cybergearrec>(Recv, 50, std::bind(&motor_drive::sub_cybergear_feedback_callback, this, std::placeholders::_1));
}

void motor_drive::sub_cybergear_feedback_callback(const r1_upper::msg::Cybergearrec::SharedPtr msg)
{
    if (msg->id == motor_id)
    {
        Last_pos = Current_pos;
        Last_speed = Current_speed;
        Last_torque = Current_torque;

        Current_pos = msg->position;
        Current_speed = msg->speed;
        Current_torque = msg->torque;

        // std::cout << "motor_id" << motor_id << "   CurrentPos:" << Current_pos << "    CurrentSpeed:" << Current_speed << "    Current_torque:" << Current_torque << std::endl;
    }
}

// 获取设备id
void motor_drive::Get_motor_id()
{
    cybergear_input.id = (master_id << 8) | (get_motor_id << 24) | (motor_id);
    cybergear_input.data[0] = 0x01;
    for (int i = 1; i < 8; i++)
    {
        cybergear_input.data[0] = 0;
    }
    pub_cybergear_input->publish(cybergear_input);
}

// 接收数据
void motor_drive::Rec()
{
    if ((reclen = VCI_Receive(VCI_USBCAN_ALYSTII, 0, ind, rec, 2500, 1)) > 0)
    {
        for (j = 0; j < reclen; j++)
        {
            if (motor_id == ((rec[j].ID << 16) >> 24))
            {
                Last_pos = Current_pos;
                Last_speed = Current_speed;
                Last_torque = Current_torque;

                Current_pos = read_angle(rec[j].Data[2 * 0], rec[j].Data[2 * 0 + 1]);
                Current_speed = read_speed(rec[j].Data[2 * 1], rec[j].Data[2 * 1 + 1]);
                Current_torque = read_torque(rec[j].Data[2 * 2], rec[j].Data[2 * 2 + 1]);

                cout << "id: " << motor_id << "  pos: " << Current_pos << "  speed: " << Current_speed << "  torque: " << Current_torque << "   j=" << j << endl;
            }
        }
    }
}

// 设置机械0位
void motor_drive::Set_zero_position()
{
    cybergear_input.id = (master_id << 8) | (set_zero_position << 24) | (motor_id);
    cybergear_input.data[0] = 0x01;
    for (int i = 1; i < 8; i++)
    {
        cybergear_input.data[i] = 0;
    }
    pub_cybergear_input->publish(cybergear_input);
    myclock.delay_us(10);

    std::cout << "set zero position  success " << std::endl;

    // 接收暂存器中可能有残留数据，强制清空暂存器并置0
    Current_pos = 0.0;
    Current_speed = 0.0;
    Current_torque = 0.0;
    std::cout << " empty!" << std::endl;
}

// 电机使能
void motor_drive::motor_enable()
{
    cybergear_input.id = (master_id << 8) | (enable << 24) | (motor_id);
    for (int i = 0; i < 8; i++)
    {
        cybergear_input.data[i] = 0;
    }
    pub_cybergear_input->publish(cybergear_input);
    myclock.delay_us(10);
}

// 电机停止
void motor_drive::motor_stop()
{
    cybergear_input.id = (master_id << 8) | (cyber_stop << 24) | (motor_id);
    for (int i = 0; i < 8; i++)
    {
        cybergear_input.data[i] = 0;
    }
    pub_cybergear_input->publish(cybergear_input);

    std::cout << "motor stop" << std::endl;
}

// 运控模式
void motor_drive::motor_operation_control(float torque, float MechPosition, float speed, float kp, float kd)
{
    // 设置位控模式
    if ((cybergear_input.id >> 24) != operation_control)
        temp_num = 0;
    if ((cybergear_input.id >> 24) != operation_control || temp_num < 5)
    {
        temp_num++;
        cybergear_input.id = (master_id << 8) | (write_parameter << 24) | (motor_id);
        cybergear_input.data[0] = (run_mode << 24) >> 24;
        cybergear_input.data[1] = run_mode >> 8;
        cybergear_input.data[2] = 0x00;
        cybergear_input.data[3] = 0x00;
        cybergear_input.data[4] = ope_ctrl;
        cybergear_input.data[5] = 0x00;
        cybergear_input.data[6] = 0x00;
        cybergear_input.data[7] = 0x00;
        pub_cybergear_input->publish(cybergear_input);
        myclock.delay_us(1000);

        cybergear_input.id = (float_to_uint(torque, T_MIN, T_MAX, 16) << 8) | (operation_control << 24) | (motor_id);
        cybergear_input.data[0] = float_to_uint(MechPosition, P_MIN, P_MAX, 16) >> 8;
        cybergear_input.data[1] = float_to_uint(MechPosition, P_MIN, P_MAX, 16);
        cybergear_input.data[2] = float_to_uint(speed, V_MIN, V_MAX, 16) >> 8;
        cybergear_input.data[3] = float_to_uint(speed, V_MIN, V_MAX, 16);
        cybergear_input.data[4] = float_to_uint(kp, KP_MIN, KP_MAX, 16) >> 8;
        cybergear_input.data[5] = float_to_uint(kp, KP_MIN, KP_MAX, 16);
        cybergear_input.data[6] = float_to_uint(kd, KD_MIN, KD_MAX, 16) >> 8;
        cybergear_input.data[7] = float_to_uint(kd, KD_MIN, KD_MAX, 16);
        pub_cybergear_input->publish(cybergear_input);
        myclock.delay_us(1000);
    }
    else
    {
        cybergear_input.id = (float_to_uint(torque, T_MIN, T_MAX, 16) << 8) | (operation_control << 24) | (motor_id);
        cybergear_input.data[0] = float_to_uint(MechPosition, P_MIN, P_MAX, 16) >> 8;
        cybergear_input.data[1] = float_to_uint(MechPosition, P_MIN, P_MAX, 16);
        cybergear_input.data[2] = float_to_uint(speed, V_MIN, V_MAX, 16) >> 8;
        cybergear_input.data[3] = float_to_uint(speed, V_MIN, V_MAX, 16);
        cybergear_input.data[4] = float_to_uint(kp, KP_MIN, KP_MAX, 16) >> 8;
        cybergear_input.data[5] = float_to_uint(kp, KP_MIN, KP_MAX, 16);
        cybergear_input.data[6] = float_to_uint(kd, KD_MIN, KD_MAX, 16) >> 8;
        cybergear_input.data[7] = float_to_uint(kd, KD_MIN, KD_MAX, 16);
        pub_cybergear_input->publish(cybergear_input);
        myclock.delay_us(10);
    }

    cout << "id: " << motor_id << "  pos: " << Current_pos << "  speed: " << Current_speed << "  torque: " << Current_torque << endl;
    // std::cout<<"operation"<<std::endl;
}

// 速度模式
void motor_drive::motor_velocity(float speed)
{
    // 设置速度模式
    std::cout << "velocity" << std::endl;
    if (cybergear_input.data[1] != (spd_ref >> 8))
    {
        cybergear_input.id = (master_id << 8) | (write_parameter << 24) | (motor_id);
        cybergear_input.data[0] = (run_mode << 24) >> 24;
        cybergear_input.data[1] = run_mode >> 8;
        cybergear_input.data[2] = 0x00;
        cybergear_input.data[3] = 0x00;
        cybergear_input.data[4] = spd;
        cybergear_input.data[5] = 0x00;
        cybergear_input.data[6] = 0x00;
        cybergear_input.data[7] = 0x00;
        pub_cybergear_input->publish(cybergear_input);
        myclock.delay_us(1000);
    }

    // 设置目标速度
    cybergear_input.id = (master_id << 8) | (write_parameter << 24) | (motor_id);
    cybergear_input.data[0] = (spd_ref << 24) >> 24;
    cybergear_input.data[1] = spd_ref >> 8;
    cybergear_input.data[2] = 0x00;
    cybergear_input.data[3] = 0x00;
    cybergear_input.data[4] = (float2hex(speed) << 24) >> 24;
    cybergear_input.data[5] = (float2hex(speed) << 16) >> 24;
    cybergear_input.data[6] = (float2hex(speed) << 8) >> 24;
    cybergear_input.data[7] = float2hex(speed) >> 24;
    pub_cybergear_input->publish(cybergear_input);
    myclock.delay_us(10);
    // std::cout<<"velocity"<<std::endl;
}

// 位置模式
void motor_drive::motor_position(float limit_speed, float position)
{

    // 设置位置模式
    cybergear_input.id = (master_id << 8) | (write_parameter << 24) | (motor_id);
    cybergear_input.data[0] = (run_mode << 24) >> 24;
    cybergear_input.data[1] = run_mode >> 8;
    cybergear_input.data[2] = 0x00;
    cybergear_input.data[3] = 0x00;
    cybergear_input.data[4] = cyber_pos;
    cybergear_input.data[5] = 0x00;
    cybergear_input.data[6] = 0x00;
    cybergear_input.data[7] = 0x00;
    pub_cybergear_input->publish(cybergear_input);
    myclock.delay_us(100);

    // 设置最大速度
    temp_speed = limit_speed;
    cybergear_input.id = (master_id << 8) | (write_parameter << 24) | (motor_id);
    cybergear_input.data[0] = (limit_spd << 24) >> 24;
    cybergear_input.data[1] = limit_spd >> 8;
    cybergear_input.data[2] = 0x00;
    cybergear_input.data[3] = 0x00;
    cybergear_input.data[4] = (float2hex(limit_speed) << 24) >> 24;
    cybergear_input.data[5] = (float2hex(limit_speed) << 16) >> 24;
    cybergear_input.data[6] = (float2hex(limit_speed) << 8) >> 24;
    cybergear_input.data[7] = float2hex(limit_speed) >> 24;
    pub_cybergear_input->publish(cybergear_input);
    myclock.delay_us(100);

    // 设置目标位置
    cybergear_input.id = (master_id << 8) | (write_parameter << 24) | (motor_id);
    cybergear_input.data[0] = (loc_ref << 24) >> 24;
    cybergear_input.data[1] = loc_ref >> 8;
    cybergear_input.data[2] = 0x00;
    cybergear_input.data[3] = 0x00;
    cybergear_input.data[4] = (float2hex(position) << 24) >> 24;
    cybergear_input.data[5] = (float2hex(position) << 16) >> 24;
    cybergear_input.data[6] = (float2hex(position) << 8) >> 24;
    cybergear_input.data[7] = float2hex(position) >> 24;
    pub_cybergear_input->publish(cybergear_input);
    myclock.delay_us(100);
    // // 设置位置模式
    // if (cybergear_input.data[1] != (loc_ref >> 8))
    // {
    //     cybergear_input.id = (master_id << 8) | (write_parameter << 24) | (motor_id);
    //     cybergear_input.data[0] = (run_mode << 24) >> 24;
    //     cybergear_input.data[1] = run_mode >> 8;
    //     cybergear_input.data[2] = 0x00;
    //     cybergear_input.data[3] = 0x00;
    //     cybergear_input.data[4] = cyber_pos;
    //     cybergear_input.data[5] = 0x00;
    //     cybergear_input.data[6] = 0x00;
    //     cybergear_input.data[7] = 0x00;
    //     pub_cybergear_input->publish(cybergear_input);
    //     myclock.delay_us(1000);
    // }

    // // 设置最大速度
    // if (temp_speed != limit_speed)
    // {
    //     temp_speed = limit_speed;
    //     cybergear_input.id = (master_id << 8) | (write_parameter << 24) | (motor_id);
    //     cybergear_input.data[0] = (limit_spd << 24) >> 24;
    //     cybergear_input.data[1] = limit_spd >> 8;
    //     cybergear_input.data[2] = 0x00;
    //     cybergear_input.data[3] = 0x00;
    //     cybergear_input.data[4] = (float2hex(limit_speed) << 24) >> 24;
    //     cybergear_input.data[5] = (float2hex(limit_speed) << 16) >> 24;
    //     cybergear_input.data[6] = (float2hex(limit_speed) << 8) >> 24;
    //     cybergear_input.data[7] = float2hex(limit_speed) >> 24;
    //     pub_cybergear_input->publish(cybergear_input);
    //     myclock.delay_us(1000);

    //     // 设置目标位置
    //     cybergear_input.id = (master_id << 8) | (write_parameter << 24) | (motor_id);
    //     cybergear_input.data[0] = (loc_ref << 24) >> 24;
    //     cybergear_input.data[1] = loc_ref >> 8;
    //     cybergear_input.data[2] = 0x00;
    //     cybergear_input.data[3] = 0x00;
    //     cybergear_input.data[4] = (float2hex(position) << 24) >> 24;
    //     cybergear_input.data[5] = (float2hex(position) << 16) >> 24;
    //     cybergear_input.data[6] = (float2hex(position) << 8) >> 24;
    //     cybergear_input.data[7] = float2hex(position) >> 24;
    //     pub_cybergear_input->publish(cybergear_input);
    //     myclock.delay_us(1000);
    // }
    // else
    // {
    //     // 设置目标位置
    //     cybergear_input.id = (master_id << 8) | (write_parameter << 24) | (motor_id);
    //     cybergear_input.data[0] = (loc_ref << 24) >> 24;
    //     cybergear_input.data[1] = loc_ref >> 8;
    //     cybergear_input.data[2] = 0x00;
    //     cybergear_input.data[3] = 0x00;
    //     cybergear_input.data[4] = (float2hex(position) << 24) >> 24;
    //     cybergear_input.data[5] = (float2hex(position) << 16) >> 24;
    //     cybergear_input.data[6] = (float2hex(position) << 8) >> 24;
    //     cybergear_input.data[7] = float2hex(position) >> 24;
    //     pub_cybergear_input->publish(cybergear_input);
    //     myclock.delay_us(10);
    // }

    cout << "id: " << motor_id << "  pos: " << Current_pos << "  speed: " << Current_speed << "  torque: " << Current_torque << endl;

    std::cout << "position" << temp_speed << std::endl;
}

// 力矩  位置 速度 kp  kd
void motor_drive::Cybergear_position_speed(float torque, float MechPosition, float speed, float kp, float kd)
{
    // if (std::fabs(Current_pos - MechPosition) > 0.2 && (!Cybergear_flag)) // 小米未到位且0.1以上
    if (fabs(Current_pos - MechPosition) > 0.2 && (!Cybergear_flag)) // 小米未到位且0.1以上
    {
        // 小米直接走运控，只是相对GO开始时间不一样
        // 速度限幅
        if (speed < 3 && speed > 0)
        {
            speed = 3;
        }
        else if (speed < 0 && speed > -3)
        {
            speed = -3;
        }
        motor_operation_control(0, 0, speed + 2.5, 0, 5);
    }
    else // 小米到位
    {
        motor_operation_control(torque, MechPosition, 0, kp, kd);
        Cybergear_flag = 1;
        // std::cout << " xiaomi  position!!!" << mycan.cybergear[0]->Current_pos << std::endl;
    }
}
