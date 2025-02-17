#include "Motor/Cybergear/can.hpp"
#include "Shoot.hpp"
#include "rclcpp/rclcpp.hpp"
#include "m3508_base.hpp"
#include "Timer.hpp"
#include "MotorPid.hpp"
#include "r1_upper/msg/cybergearsend.hpp"
#include "r1_upper/msg/cybergearrec.hpp"

MYTIMER receive_hand_timer;
float SpeedExpected = 0;
float CurrentExpected = 0;
int32_t PWM = 0;
float sub_pos[2] = {0};
float sub_vel[2] = {0};
float sub_cur[2] = {0};
float sub_pre_cur[2] = {0};
int sub_motor_mode[2] = {0};
int cybergear_is_receive = 0;
VCI_CAN_OBJ CmdSend[1], CmdSend1[1], CybergearCmdSend[1], CybergearCmdSend1[1];
VCI_CAN_OBJ DataRecv[2500];
r1_upper::msg::M3508rec m3508_feedback[2];
r1_upper::msg::Cybergearrec cybergear_feedback[2];

// 创建一个类节点，名字叫做CustomerNode,继承自Node.
class receive_hand_exec_node : public rclcpp::Node
{
private:
public:
    rclcpp::Subscription<r1_upper::msg::M3508send>::SharedPtr sub_m3508_input;
    rclcpp::Publisher<r1_upper::msg::M3508rec>::SharedPtr pub_m3508_feedback;
    rclcpp::Subscription<r1_upper::msg::Cybergearsend>::SharedPtr sub_cybergear_input;
    rclcpp::Publisher<r1_upper::msg::Cybergearrec>::SharedPtr pub_cybergear_feedback;

    static void sub_m3508_input_callback(const r1_upper::msg::M3508send::SharedPtr msg)
    {
        if (msg->id < 0x205)
        {
            CmdSend[0].ID = 0x200;
            CmdSend[0].SendType = 1;
            CmdSend[0].RemoteFlag = 0;
            CmdSend[0].ExternFlag = 0;
            CmdSend[0].DataLen = 8;
            sub_motor_mode[0] = msg->mode;
            MotorPositionPid1.Kp = msg->kp;
            MotorPositionPid1.Ki = msg->ki;
            MotorPositionPid1.Kd = msg->kd;
            MotorPositionPid1.LimitOutput = msg->limit_output;
            for (int i = 0; i < CmdSend[0].DataLen; ++i)
            {
                CmdSend[0].Data[i] = 0;
            }
            if (msg->mode == M3508_POS_MODE)
            {
                SpeedExpected = ClassicPidRegulate(msg->pos, m3508_feedback[0].pos, &MotorPositionPid1);
                CurrentExpected = ClassicPidRegulate(SpeedExpected, m3508_feedback[0].speed, &MotorSpeedPid1);
                // std::cout<<CurrentExpected+msg->current<<"   "<<CurrentExpected<<std::endl;
                std::cout << "   " << msg->current << std::endl;
                PWM = int16_t(CurrentExpected);
                sub_pre_cur[0] = msg->current;
                sub_pos[0] = msg->pos;
                std::cout << sub_pre_cur[0] << "   " << msg->current << std::endl;
            }
            else if (sub_motor_mode[0] == M3508_SPEED_MODE)
            {
                CurrentExpected = ClassicPidRegulate(msg->speed, m3508_feedback[0].speed, &MotorSpeedPid1);
                PWM = int16_t(CurrentExpected);
                sub_vel[0] = msg->speed;
            }
            else if (sub_motor_mode[0] == M3508_CURRENT_MODE)
            {
                PWM = int16_t(msg->current);
                sub_cur[0] = msg->current;
            }
            CmdSend[0].Data[0] = PWM >> 8;
            CmdSend[0].Data[1] = PWM & 0xff;
        }
        else
        {
            CmdSend1[0].ID = 0x1FF;
            CmdSend1[0].SendType = 1;
            CmdSend1[0].RemoteFlag = 0;
            CmdSend1[0].ExternFlag = 0;
            CmdSend1[0].DataLen = 8;
            sub_motor_mode[1] = msg->mode;
            MotorPositionPid2.Kp = msg->kp;
            MotorPositionPid2.Ki = msg->ki;
            MotorPositionPid2.Kd = msg->kd;
            MotorPositionPid2.LimitOutput = msg->limit_output;
            for (int i = 0; i < CmdSend1[0].DataLen; ++i)
            {
                CmdSend1[0].Data[i] = 0;
            }
            if (msg->mode == M3508_POS_MODE)
            {
                SpeedExpected = ClassicPidRegulate(msg->pos, m3508_feedback[1].pos, &MotorPositionPid2);
                CurrentExpected = ClassicPidRegulate(SpeedExpected, m3508_feedback[1].speed, &MotorSpeedPid2);
                PWM = int16_t(CurrentExpected);
                sub_pos[1] = msg->pos;
                sub_pre_cur[1] = msg->current;
            }
            else if (sub_motor_mode[1] == M3508_SPEED_MODE)
            {
                CurrentExpected = ClassicPidRegulate(msg->speed, m3508_feedback[1].speed, &MotorSpeedPid2);
                PWM = int16_t(CurrentExpected);
                sub_vel[1] = msg->speed;
            }
            else if (sub_motor_mode[1] == M3508_CURRENT_MODE)
            {
                PWM = int16_t(msg->current);
                sub_cur[1] = msg->current;
            }
            CmdSend1[0].Data[0] = msg->pwm >> 8;
            CmdSend1[0].Data[1] = msg->pwm & 0xff;
        }
    }

    static void sub_cybergear_input_callback(const r1_upper::msg::Cybergearsend::SharedPtr msg)
    {
        // cybergear_is_receive = 1;
        if (((msg->id<<24)>>24) == 0x01)
        {
            CybergearCmdSend[0].ID = msg->id;
            CybergearCmdSend[0].SendType = 0;
            CybergearCmdSend[0].RemoteFlag = 0;
            CybergearCmdSend[0].ExternFlag = 1;
            CybergearCmdSend[0].DataLen = 8;
            // std::cout << "data:" << msg->id << std::endl;
            for (int i = 0; i < 8; i++)
            {
                CybergearCmdSend[0].Data[i] = msg->data[i];
                // frame[0].Data[i] = 0;
            }
        }
        else if (((msg->id<<24)>>24) == 0x02)
        {
            CybergearCmdSend1[0].ID = msg->id;
            CybergearCmdSend1[0].SendType = 0;
            CybergearCmdSend1[0].RemoteFlag = 0;
            CybergearCmdSend1[0].ExternFlag = 1;
            CybergearCmdSend1[0].DataLen = 8;
            // std::cout << "data:" << msg->id << std::endl;
            for (int i = 0; i < 8; i++)
            {
                CybergearCmdSend1[0].Data[i] = msg->data[i];
                // frame[0].Data[i] = 0;
            }
        }
    }

    // 构造函数,第一个参数为节点名称
    receive_hand_exec_node(std::string name) : Node(name)
    {
        pub_m3508_feedback = this->create_publisher<r1_upper::msg::M3508rec>("/m3508_receive", 50);
        sub_m3508_input = this->create_subscription<r1_upper::msg::M3508send>("/m3508_send", 50, receive_hand_exec_node::sub_m3508_input_callback);
        pub_cybergear_feedback = this->create_publisher<r1_upper::msg::Cybergearrec>("/cybergear_receive", 50);
        sub_cybergear_input = this->create_subscription<r1_upper::msg::Cybergearsend>("/cybergear_send", 50, receive_hand_exec_node::sub_cybergear_input_callback);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    mycan.CAN_open();
    MotorPidInit();
    auto node = std::make_shared<receive_hand_exec_node>("Receive_hand_exe");
    rclcpp::Rate loop_rate(1000); // 750
    int reclen = 0;
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        if (sub_motor_mode[0] == M3508_POS_MODE)
        {
            SpeedExpected = ClassicPidRegulate(sub_pos[0], m3508_feedback[0].pos, &MotorPositionPid1);
            CurrentExpected = ClassicPidRegulate(SpeedExpected, m3508_feedback[0].speed, &MotorSpeedPid1);
            if (fabs(CurrentExpected + sub_pre_cur[0]) < 16000)
            {
                PWM = int16_t(CurrentExpected + sub_pre_cur[0]);
            }
            else
            {
                PWM = int16_t((CurrentExpected + sub_pre_cur[0]) / fabs(CurrentExpected + sub_pre_cur[0]) * 16000.0);
            }
        }
        else if (sub_motor_mode[0] == M3508_SPEED_MODE)
        {
            CurrentExpected = ClassicPidRegulate(sub_vel[0], m3508_feedback[0].speed, &MotorSpeedPid1);
            PWM = int16_t(CurrentExpected);
        }
        else if (sub_motor_mode[0] == M3508_CURRENT_MODE)
        {
            PWM = int16_t(sub_cur[0]);
        }
        CmdSend[0].Data[0] = PWM >> 8;
        CmdSend[0].Data[1] = PWM & 0xff;

        if (sub_motor_mode[1] == M3508_POS_MODE)
        {
            SpeedExpected = ClassicPidRegulate(sub_pos[1], m3508_feedback[1].pos, &MotorPositionPid2);
            CurrentExpected = ClassicPidRegulate(SpeedExpected, m3508_feedback[1].speed, &MotorSpeedPid2);
            if (fabs(CurrentExpected + sub_pre_cur[1]) < 16000)
            {
                PWM = int16_t(CurrentExpected + sub_pre_cur[1]);
            }
            else
            {
                PWM = int16_t((CurrentExpected + sub_pre_cur[1]) / fabs(CurrentExpected + sub_pre_cur[1]) * 16000.0);
            }
        }
        else if (sub_motor_mode[1] == M3508_SPEED_MODE)
        {
            CurrentExpected = ClassicPidRegulate(sub_vel[1], m3508_feedback[1].speed, &MotorSpeedPid2);
            PWM = int16_t(CurrentExpected);
        }
        else if (sub_motor_mode[1] == M3508_CURRENT_MODE)
        {
            PWM = int16_t(sub_cur[1]);
        }
        CmdSend1[0].Data[0] = PWM >> 8;
        CmdSend1[0].Data[1] = PWM & 0xff;

        // 运行节点，并检测退出信号
        // std::cout << "  PWM" << (CmdSend[0].Data[0]) << (CmdSend[0].Data[1]) << std::endl;
        // std::cout << "id" << m3508_feedback.id << "   pos" << m3508_feedback.pos << "   speed" << m3508_feedback.speed << "   current" << m3508_feedback.current << std::endl;
        // if (cybergear_is_receive)
        // {
        VCI_Transmit(VCI_USBCAN_ALYSTII, 0, 0, CybergearCmdSend, 1);
        receive_hand_timer.delay_us(400);
        VCI_Transmit(VCI_USBCAN_ALYSTII, 0, 0, CybergearCmdSend1, 1);
        receive_hand_timer.delay_us(400);
        // cybergear_is_receive = 0;
        // VCI_Transmit(VCI_USBCAN_ALYSTII, 0, 0, CmdSend, 1);
        // receive_hand_timer.delay_us(333);
        // VCI_Transmit(VCI_USBCAN_ALYSTII, 0, 0, CmdSend1, 1);
        // receive_hand_timer.delay_us(333);
        // }
        // else
        // {
        //     VCI_Transmit(VCI_USBCAN_ALYSTII, 0, 0, CmdSend, 1);
        //     receive_hand_timer.delay_us(500);
        //     VCI_Transmit(VCI_USBCAN_ALYSTII, 0, 0, CmdSend1, 1);
        //     receive_hand_timer.delay_us(500);
        // }

        if ((reclen = VCI_Receive(VCI_USBCAN_ALYSTII, 0, 0, DataRecv, 2500, 1)) > 0)
        {
            for (int j = 0; j < reclen; j++)
            {
                if (DataRecv[j].ID == mycan.m3508[0]->motor_id)
                {
                    if (mycan.m3508[0]->motortypedef.PosPre == 0 && mycan.m3508[0]->motortypedef.PosNow == 0)
                        mycan.m3508[0]->motortypedef.PosPre = mycan.m3508[0]->motortypedef.PosNow = (short)(DataRecv[j].Data[0] << 8 | DataRecv[j].Data[1]);
                    else
                    {
                        mycan.m3508[0]->motortypedef.PosPre = mycan.m3508[0]->motortypedef.PosNow;
                        mycan.m3508[0]->motortypedef.PosNow = (short)(DataRecv[j].Data[0] << 8 | DataRecv[j].Data[1]);
                    }
                    mycan.m3508[0]->motortypedef.PositionMeasure += mycan.m3508[0]->Get_RM3508_Distance(mycan.m3508[0]->motortypedef);

                    m3508_feedback[0].id = mycan.m3508[0]->motor_id;
                    m3508_feedback[0].pos = mycan.m3508[0]->motortypedef.PositionMeasure;
                    m3508_feedback[0].speed = ((float)(short)(DataRecv[j].Data[2] << 8 | DataRecv[j].Data[3])) / mycan.m3508[0]->M3508_REDUCE_RATIO;
                    m3508_feedback[0].current = (float)(short)(DataRecv[j].Data[4] << 8 | DataRecv[j].Data[5]);
                    node->pub_m3508_feedback->publish(m3508_feedback[0]);
                    // std::cout << "   m3508_feedback.id" << DataRecv[j].ID << "   m3508_feedback.pos" << m3508_feedback[0].pos << std::endl;
                }
                else if (DataRecv[j].ID == mycan.m3508[1]->motor_id)
                {
                    if (mycan.m3508[1]->motortypedef.PosPre == 0 && mycan.m3508[1]->motortypedef.PosNow == 0)
                        mycan.m3508[1]->motortypedef.PosPre = mycan.m3508[1]->motortypedef.PosNow = (short)(DataRecv[j].Data[0] << 8 | DataRecv[j].Data[1]);
                    else
                    {
                        mycan.m3508[1]->motortypedef.PosPre = mycan.m3508[1]->motortypedef.PosNow;
                        mycan.m3508[1]->motortypedef.PosNow = (short)(DataRecv[j].Data[0] << 8 | DataRecv[j].Data[1]);
                    }
                    mycan.m3508[1]->motortypedef.PositionMeasure += mycan.m3508[1]->Get_RM3508_Distance(mycan.m3508[1]->motortypedef);

                    m3508_feedback[1].id = mycan.m3508[1]->motor_id;
                    m3508_feedback[1].pos = mycan.m3508[1]->motortypedef.PositionMeasure;
                    m3508_feedback[1].speed = ((float)(short)(DataRecv[j].Data[2] << 8 | DataRecv[j].Data[3])) / mycan.m3508[1]->M3508_REDUCE_RATIO;
                    m3508_feedback[1].current = (float)(short)(DataRecv[j].Data[4] << 8 | DataRecv[j].Data[5]);
                    node->pub_m3508_feedback->publish(m3508_feedback[1]);
                    // std::cout << "   m3508_feedback.id" << DataRecv[j].ID << "   m3508_feedback.pos" << m3508_feedback[1].pos << std::endl;
                }
                else if (((DataRecv[j].ID << 16) >> 24) == mycan.cybergear[0]->motor_id)
                {
                    cybergear_feedback[0].id = mycan.cybergear[0]->motor_id;
                    cybergear_feedback[0].position = mycan.cybergear[0]->read_angle(DataRecv[j].Data[2 * 0], DataRecv[j].Data[2 * 0 + 1]);
                    cybergear_feedback[0].speed = mycan.cybergear[0]->read_speed(DataRecv[j].Data[2 * 1], DataRecv[j].Data[2 * 1 + 1]);
                    cybergear_feedback[0].speed = mycan.cybergear[0]->read_torque(DataRecv[j].Data[2 * 2], DataRecv[j].Data[2 * 2 + 1]);
                    node->pub_cybergear_feedback->publish(cybergear_feedback[0]);
                }
                else if (((DataRecv[j].ID << 16) >> 24) == mycan.cybergear[1]->motor_id)
                {
                    cybergear_feedback[1].id = mycan.cybergear[1]->motor_id;
                    cybergear_feedback[1].position = mycan.cybergear[1]->read_angle(DataRecv[j].Data[2 * 0], DataRecv[j].Data[2 * 0 + 1]);
                    cybergear_feedback[1].speed = mycan.cybergear[1]->read_speed(DataRecv[j].Data[2 * 1], DataRecv[j].Data[2 * 1 + 1]);
                    cybergear_feedback[1].speed = mycan.cybergear[1]->read_torque(DataRecv[j].Data[2 * 2], DataRecv[j].Data[2 * 2 + 1]);
                    node->pub_cybergear_feedback->publish(cybergear_feedback[1]);
                }
            }
        }
        loop_rate.sleep();
    }
    return 0;
}