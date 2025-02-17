#include <unistd.h>
#include "Motor/Go/serialPort/SerialPort.h"
#include "Motor/Go/unitreeMotor/unitreeMotor.h"
#include "Motor/Go/GO_base.hpp"
#include "Timer.hpp"
#include <cmath>

MYTIMER Timer;

void GO_Drive::GO_init(int id, std::string temp_serial_usb)
{
    GO_Drive::ID = id;
    std::cout << "GO-Motor  Init Success    ID: " << ID << std::endl;
    GO_Drive::cmd.motorType = MotorType::GO_M8010_6;
    GO_Drive::data.motorType = MotorType::GO_M8010_6;
    GO_Drive::cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
    GO_Drive::cmd.id = ID;
    GO_Drive::cmd.kp = 0.0;
    GO_Drive::cmd.kd = 0.0;
    GO_Drive::cmd.q = 0.0;
    GO_Drive::cmd.dq = 0.0;
    GO_Drive::cmd.tau = 0.0;

    serial = new SerialPort(temp_serial_usb);
    serial->sendRecv(&cmd, &data);

    std::cout << std::endl;
    std::cout << "id" << "position_Init:     " << GO_Drive::data.q << std::endl;
    GO_Drive::Zero_pos = GO_Drive::data.q;
}

// 速度指令，运行速度和运行时间（ms）
void GO_Drive::GO_Velocity_Time(float vel, int time)
{

    for (int j = 0; j < time; j++)
    {
        Pos_now = (data.q - Zero_pos) / queryGearRatio(MotorType::GO_M8010_6);

        GO_Drive::cmd.motorType = MotorType::GO_M8010_6;
        GO_Drive::data.motorType = MotorType::GO_M8010_6;
        GO_Drive::cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
        GO_Drive::cmd.id = ID;
        GO_Drive::cmd.kp = 0.0;
        GO_Drive::cmd.kd = 0.01;
        GO_Drive::cmd.q = 0.0;
        GO_Drive::cmd.dq = -vel * queryGearRatio(MotorType::GO_M8010_6);
        GO_Drive::cmd.tau = 0.0;

        serial->sendRecv(&cmd, &data);

        std::cout << std::endl;
        std::cout << "GO-Motor Mode:  Velocity      Paras:  " << vel << std::endl;
        std::cout << "motor.q: " << GO_Drive::data.q << std::endl;
        std::cout << "motor.temp: " << GO_Drive::data.temp << std::endl;
        std::cout << "motor.W: " << GO_Drive::data.dq << std::endl;
        std::cout << "motor.merror: " << GO_Drive::data.merror << std::endl;
        std::cout << std::endl;

        Timer.delay_sec(1);
        // Timer.delay_sec(1);
    }

    // 停止
    Pos_now = (data.q - Zero_pos) / queryGearRatio(MotorType::GO_M8010_6);

    cmd.motorType = MotorType::GO_M8010_6;
    data.motorType = MotorType::GO_M8010_6;
    cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
    cmd.id = ID;
    cmd.kp = 0.0;
    cmd.kd = 0.0;
    cmd.q = 0.0;
    cmd.dq = 0.0;
    cmd.tau = 0.0;
    serial->sendRecv(&cmd, &data);
    std::cout << std::endl;
    std::cout << "GO-Motor Mode:   stop  " << std::endl;
    std::cout << "motor.q: " << data.q << std::endl;
    std::cout << "motor.temp: " << data.temp << std::endl;
    std::cout << "motor.W: " << data.dq << std::endl;
    std::cout << "motor.merror: " << data.merror << std::endl;
    std::cout << std::endl;
}

void GO_Drive::GO_Velocity(float vel, float kd)
{
    Pos_now = (data.q - Zero_pos) / queryGearRatio(MotorType::GO_M8010_6);
    GO_Drive::cmd.motorType = MotorType::GO_M8010_6;
    GO_Drive::data.motorType = MotorType::GO_M8010_6;
    GO_Drive::cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
    GO_Drive::cmd.id = ID;
    GO_Drive::cmd.kp = 0.0;
    GO_Drive::cmd.kd = kd;
    GO_Drive::cmd.q = 0.0;
    GO_Drive::cmd.dq = vel * queryGearRatio(MotorType::GO_M8010_6);
    GO_Drive::cmd.tau = 0.0;

    serial->sendRecv(&cmd, &data);
}

// 位置指令
void GO_Drive::GO_Position(float pos, float kp, float kd)
{

    Pos_now = (data.q - Zero_pos) / queryGearRatio(MotorType::GO_M8010_6);
    cmd.motorType = MotorType::GO_M8010_6;
    data.motorType = MotorType::GO_M8010_6;
    cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
    cmd.id = ID;
    cmd.kp = kp; // 0.02
    cmd.kd = kd;
    cmd.q = Zero_pos + pos * queryGearRatio(MotorType::GO_M8010_6);
    cmd.dq = 0.0;
    cmd.tau = 0.0;
    serial->sendRecv(&cmd, &data);
    // std::cout << std::endl;
    std::cout << "GO-Motor Mode:  Position      Paras:  " << pos << "    " << kp << std::endl;
    // std::cout << "motor.q: " << data.q << std::endl;
    // std::cout << "motor.temp: " << data.temp << std::endl;
    // std::cout << "motor.W: " << data.dq << std::endl;
    // std::cout << "motor.merror: " << data.merror << std::endl;
    // std::cout << std::endl;
}

void GO_Drive::GO_Dumping(float dump)
{

    Pos_now = (data.q - Zero_pos) / queryGearRatio(MotorType::GO_M8010_6);

    cmd.motorType = MotorType::GO_M8010_6;
    data.motorType = MotorType::GO_M8010_6;
    cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
    cmd.id = ID;
    cmd.kp = 0.0;
    cmd.kd = dump;
    cmd.q = 0.0;
    cmd.dq = 0.0;
    cmd.tau = 0.0;
    serial->sendRecv(&cmd, &data);
    std::cout << std::endl;
    std::cout << "GO-Motor Mode:  Dumping      Paras:  " << dump << std::endl;
    std::cout << "motor.q: " << data.q << std::endl;
    std::cout << "motor.temp: " << data.temp << std::endl;
    std::cout << "motor.W: " << data.dq << std::endl;
    std::cout << "motor.merror: " << data.merror << std::endl;
    std::cout << std::endl;
}

void GO_Drive::GO_moment(float tau)

{
    Pos_now = (data.q - Zero_pos) / queryGearRatio(MotorType::GO_M8010_6);

    cmd.motorType = MotorType::GO_M8010_6;
    data.motorType = MotorType::GO_M8010_6;
    cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
    cmd.id = ID;
    cmd.kp = 0.0;
    cmd.kd = 0.0;
    cmd.q = 0.0;
    cmd.dq = 0.0;
    cmd.tau = tau;
    serial->sendRecv(&cmd, &data);
    std::cout << std::endl;
    std::cout << "GO-Motor Mode:  moment      Paras:  " << tau << std::endl;
    std::cout << "motor.q: " << data.q << std::endl;
    std::cout << "motor.temp: " << data.temp << std::endl;
    std::cout << "motor.W: " << data.dq << std::endl;
    std::cout << "motor.merror: " << data.merror << std::endl;
    std::cout << std::endl;
}

void GO_Drive::GO_Stop()
{
    Pos_now = (data.q - Zero_pos) / queryGearRatio(MotorType::GO_M8010_6);
    cmd.motorType = MotorType::GO_M8010_6;
    data.motorType = MotorType::GO_M8010_6;
    cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
    cmd.id = ID;
    cmd.kp = 0.0;
    cmd.kd = 0.0;
    cmd.q = 0.0;
    cmd.dq = 0.0;
    cmd.tau = 0.0;
    serial->sendRecv(&cmd, &data);
    // std::cout <<  std::endl;
    // std::cout<< "GO-Motor Mode :   Stop"<<std::endl;
    std::cout << "id" << GO_Drive::ID << "motor.q: " << Pos_now << std::endl;
    // // std::cout <<  "motor.temp: "   << data.temp   <<  std::endl;
    // std::cout <<  "motor.W: "      << data.dq      <<  std::endl;
    // // std::cout <<  "motor.merror: " << data.merror <<  std::endl;
    std::cout << std::endl;
}

void GO_Drive::GO_Movement(float kp, float kd, float q, float dq, float tau)
{
    Pos_now = (data.q - Zero_pos) / queryGearRatio(MotorType::GO_M8010_6);
    cmd.motorType = MotorType::GO_M8010_6;
    data.motorType = MotorType::GO_M8010_6;
    cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
    cmd.id = ID;
    cmd.kp = kp;
    cmd.kd = kd;
    cmd.q = Zero_pos + q * queryGearRatio(MotorType::GO_M8010_6);
    cmd.dq = dq * queryGearRatio(MotorType::GO_M8010_6);
    cmd.tau = tau;
    serial->sendRecv(&cmd, &data);
    // std::cout << std::endl;
    // std::cout << "GO-Motor Mode :   Movement  paras:" << kp << "   " << kd << "  " << q << "  " << dq << "  " << tau << std::endl;
    // std::cout << "motor.q: " << (data.q - Zero_pos) / queryGearRatio(MotorType::GO_M8010_6) << std::endl;
    // std::cout << "motor.temp: " << data.temp << std::endl;
    // std::cout << "motor.W: " << data.dq << std::endl;
    // std::cout << "motor.merror: " << data.merror << std::endl;
    // std::cout << std::endl;
}

void GO_Drive::GO_position_speed(float GO_kp, float GO_kd, float GO_q, float GO_dq, float delay_flag_q)
{
    Pos_now = (data.q - Zero_pos) / queryGearRatio(MotorType::GO_M8010_6);
    // GO到位之前跑速度环
    if (fabs(Pos_now - GO_q) > 0.5)
    {
        GO_Drive::GO_Velocity(GO_dq, GO_kd);
    }
    else // GO到位用力矩模式锁住
    {
        cmd.id = ID;
        cmd.kp = GO_kp;
        cmd.q = Zero_pos + GO_q * queryGearRatio(MotorType::GO_M8010_6);
        cmd.dq = GO_dq;
        cmd.tau = 0;
        cmd.kd = GO_kd;
        serial->sendRecv(&cmd, &data);

        GO_flag = 1;
    }
    // 下一个电机启动标志位
    if (fabs(Pos_now) > fabs(delay_flag_q))
    {
        delay_flag = 1;
    }
}

void GO_Drive::GO_shoot(float GO_kp, float GO_kd, float GO_q, float GO_dq, float delay_flag_q, int mode)
{
    Pos_now = (data.q - Zero_pos) / queryGearRatio(MotorType::GO_M8010_6);
    // GO到位之前跑速度环
    if (mode == 0)
    {
        if (fabs(Pos_now - GO_q) > 0.5)
        {
            GO_Drive::GO_Velocity(GO_dq, GO_kd);
        }
        else if (fabs(Pos_now - GO_q) > 0.1)
        {
            GO_Drive::GO_Velocity((fabs(Pos_now - GO_q)) / 0.5 * GO_dq, GO_kd);
        }
        else // GO到位用力矩模式锁住
        {
            cmd.id = ID;
            cmd.kp = GO_kp*0.5;
            cmd.q = Zero_pos + GO_q * queryGearRatio(MotorType::GO_M8010_6);
            cmd.dq = GO_dq;
            cmd.tau = 0;
            cmd.kd = GO_kd*0.5;
            serial->sendRecv(&cmd, &data);

            GO_flag = 1;
        }
    }
    else
    {
        if (fabs(Pos_now - GO_q) > 0.2)
        {
            GO_Drive::GO_Velocity(GO_dq, GO_kd);
        }
        else // GO到位用力矩模式锁住
        {
            cmd.id = ID;
            cmd.kp = GO_kp;
            cmd.q = Zero_pos + GO_q * queryGearRatio(MotorType::GO_M8010_6);
            cmd.dq = GO_dq;
            cmd.tau = 0;
            cmd.kd = GO_kd;
            serial->sendRecv(&cmd, &data);

            GO_flag = 1;
        }
    }
    // 下一个电机启动标志位
    if (fabs(Pos_now) > fabs(delay_flag_q))
    {
        delay_flag = 1;
    }
}
