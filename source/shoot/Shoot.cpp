#include <cmath>
#include "Timer.hpp"
#include "Motor/Cybergear/can.hpp"
#include "Shoot.hpp"

CAN mycan;
GO_Drive MyGO[4];
MYTIMER Shoot_Timer;
float t_temp_speed = 0.0;

// 机械臂初始化 电机初始化以及锁在原位
void MANIPULATOR::Manipulator_Init()
{
    mycan.m3508[0]->motor_3508_init("/m3508_send", "/m3508_receive", "m3508_1");
    Shoot_Timer.delay_ms(500);
    mycan.m3508[1]->motor_3508_init("/m3508_send", "/m3508_receive", "m3508_2");
    Shoot_Timer.delay_ms(500);
    mycan.cybergear[0]->cybergear_init("/cybergear_send", "/cybergear_receive", "cybergear_1");
    Shoot_Timer.delay_ms(500);
    mycan.cybergear[1]->cybergear_init("/cybergear_send", "/cybergear_receive", "cybergear_2");
    Shoot_Timer.delay_ms(500);
    MyGO[0].GO_init(1, UPPER_ARM_PORT);
    Shoot_Timer.delay_ms(1);
    MyGO[1].GO_init(2, UNDER_ARM_PORT);
    Shoot_Timer.delay_ms(1);
    MyGO[2].GO_init(3, WRIST_PORT);
    Shoot_Timer.delay_ms(1);
    MyGO[3].GO_init(4, RECEIVE_BALL_PORT);
    Shoot_Timer.delay_ms(1);

    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 5; j++)
        {
            mycan.cybergear[i]->Set_zero_position();
            Shoot_Timer.delay_ms(3);
            mycan.cybergear[i]->motor_enable();
            Shoot_Timer.delay_ms(3);
            mycan.cybergear[i]->motor_position(6, 0.0);
            Shoot_Timer.delay_ms(3);
        }
    }

    MyGO[0].GO_Movement(5.0, 0.1, 0.0, 0.0, 0); // 正
    MyGO[1].GO_Movement(5.0, 0.1, 0.0, 0.0, 0); // 负
    MyGO[2].GO_Position(0.0, 0.5, 0.01);        // 正
    float temp_start_i = 0;
    while (temp_start_i <= Common_Receive_Ball_Pos)
    {
        MyGO[3].GO_Movement(2.5, 0.05, temp_start_i, 0.0, 0); // 负
        temp_start_i = temp_start_i + 0.01;
        Shoot_Timer.delay_ms(5);
    }
}

// 读电机数据专用
void MANIPULATOR::Read_position()
{
    while (1)
    {
        MyGO[0].GO_Stop();
        MyGO[1].GO_Stop();
        Shoot_Timer.delay_ms(100);
    }
}

// 运球
void MANIPULATOR::Dribble()
{
    // 机械臂先到预定位置
    if (!Dribble_Pre_flag)
    {
        MyGO[0].GO_Movement(5, 0.1, Dribble_Pre_Current_Time / float(Dribble_Pre_Total_Time) * DRIBBLE_PRE_GO1_POSITION, 3, 0);
        MyGO[1].GO_Movement(5, 0.1, Dribble_Pre_Current_Time / float(Dribble_Pre_Total_Time) * DRIBBLE_PRE_GO2_POSITION, -3, 0);
        MyGO[2].GO_Position(Dribble_Pre_Current_Time / float(Dribble_Pre_Total_Time) * DRIBBLE_PRE_GO3_POSITION, 5.0, 0.1);
        // MyGO[2].GO_Velocity(1 + Dribble_Pre_Current_Time / float(Dribble_Pre_Total_Time) * 2.0, 0.1);
        // if (fabs(MyGO[2].Pos_now) > fabs(DRIBBLE_PRE_GO3_POSITION))
        if (Dribble_Pre_Current_Time == Dribble_Pre_Total_Time)
        {
            Dribble_Pre_flag = 1;
            std::cout << "dribble preparation finish" << std::endl;
        }
        else
        {
            Shoot_Timer.delay_ms(4);
            Dribble_Pre_Current_Time++;
        }
    }
    // 然后竖直向下拍球
    else
    {
        if (!Dribble_flag)
        {
            MyGO[0].GO_Movement(50, 0.15, DRIBBLE_PRE_GO1_POSITION, 3, 0);
            MyGO[1].GO_Movement(80, 0.15, DRIBBLE_PRE_GO2_POSITION, -3, 0);
            // MyGO[2].GO_Velocity(8.5 + Dribble_Current_Time / float(Dribble_Total_Time) * 5.1, 0.1);
            // MyGO[2].GO_Velocity(8.5 + Dribble_Current_Time / float(Dribble_Total_Time) * 5.1, 0.1);
            MyGO[2].GO_Position(DRIBBLE_GO3_POSITION, 1.1, 0.015);
            Dribble_Current_Time++;
            // if (fabs(MyGO[2].Pos_now) > fabs(DRIBBLE_GO3_POSITION))
            if (Dribble_Current_Time == Dribble_Total_Time)
            {
                // MyGO[2].GO_position_speed(5.0,0.2,DRIBBLE_GO3_POSITION / 3,20.0,0.0);
                // for (int i = 100; i >= 0; i--)
                // {
                //     // MyGO[1].GO_Movement(5.0, 0.1, DRIBBLE_PRE_GO2_POSITION / 2 + float(i / 100.0) * DRIBBLE_PRE_GO2_POSITION / 2, 0.0, 0);
                //     MyGO[2].GO_Position(DRIBBLE_GO3_POSITION / 3 + float(i / 100.0) * DRIBBLE_GO3_POSITION * 2 / 3, 0.5, 0.01);
                //     Shoot_Timer.delay_ms(0.5);
                // }
                Shoot_Timer.delay_ms(150);
                MyGO[2].GO_Position(DRIBBLE_GO3_POSITION * 0.7, 0.5, 0.01);
                Dribble_flag = 1;
                for (int i = 0; i < 16; i++)
                {
                    Dribble_Receive_Ball();
                }
                Shoot_Timer.delay_ms(500);
                // // MyGO[2].GO_Position(DRIBBLE_GO3_POSITION * 0.4, 0.5, 0.01);
                // Shoot_Timer.delay_ms(500);
                std::cout << "dribble finish" << std::endl;
            }
            Shoot_Timer.delay_ms(1.5);
        }
    }
}

// 运球复位
void MANIPULATOR::Dribble_Reset_Position()
{
    if (!Dribble_Reset_flag)
    {
        if (Dribble_Reset_Current_Time < Dribble_Reset_Total_Time / 3)
        {
            MyGO[1].GO_Movement(5.0, 0.1, (Dribble_Reset_Current_Time) / float(Dribble_Reset_Total_Time / 3) * (-0.08), 0.0, 0);
        }
        else
        {
            MyGO[0].GO_Movement(5.0, 0.1, (Dribble_Reset_Current_Time - Dribble_Reset_Total_Time / 3) / float(Dribble_Reset_Total_Time * 2 / 3) * DRIBBLE_PRE_GO1_POSITION, 0.0, 0);
            MyGO[1].GO_Movement(5.0, 0.1, (Dribble_Reset_Current_Time - Dribble_Reset_Total_Time / 3) / float(Dribble_Reset_Total_Time * 2 / 3) * (DRIBBLE_PRE_GO2_POSITION + 0.08) - 0.08, 0.0, 0);
            // MyGO[1].GO_Movement(5.0, 0.1, (Dribble_Reset_Current_Time - Dribble_Reset_Total_Time / 3) / float(Dribble_Reset_Total_Time * 2 / 3) * (DRIBBLE_PRE_GO2_POSITION / 2 + 0.08) - 0.08, 0.0, 0);
            // MyGO[2].GO_Position((Dribble_Reset_Current_Time - Dribble_Reset_Total_Time / 3) / float(Dribble_Reset_Total_Time * 2 / 3) * (DRIBBLE_GO3_POSITION / 3), 0.5, 0.01);
            MyGO[2].GO_Position((Dribble_Reset_Current_Time - Dribble_Reset_Total_Time / 3) / float(Dribble_Reset_Total_Time * 2 / 3) * (DRIBBLE_GO3_POSITION * 0.7), 0.5, 0.01);
        }
        Dribble_Reset_Current_Time--;
        if (Dribble_Reset_Current_Time == 0)
        {
            Dribble_Reset_flag = 1;
            std::cout << "dribble reset finish" << std::endl;
        }
        Shoot_Timer.delay_ms(5);
    }
}

// // 运球
// void MANIPULATOR::Dribble()
// {
//     // 机械臂先到预定位置
//     if (!Dribble_Pre_flag)
//     {
//         MyGO[0].GO_Movement(5, 0.1, Dribble_Pre_Current_Time / float(Dribble_Pre_Total_Time) * DRIBBLE_PRE_GO1_POSITION, 3, 0);
//         MyGO[1].GO_Movement(5, 0.1, Dribble_Pre_Current_Time / float(Dribble_Pre_Total_Time) * DRIBBLE_PRE_GO2_POSITION, -3, 0);
//         MyGO[2].GO_Movement(5, 0.1, Dribble_Pre_Current_Time / float(Dribble_Pre_Total_Time) * DRIBBLE_PRE_GO3_POSITION, -3, 0);
//         if (Dribble_Pre_Current_Time == Dribble_Pre_Total_Time)
//         {
//             Dribble_Pre_flag = 1;
//             std::cout << "dribble preparation finish" << std::endl;
//         }
//         else
//         {
//             Shoot_Timer.delay_ms(4);
//             Dribble_Pre_Current_Time++;
//         }
//     }
//     // 然后竖直向下拍球
//     else
//     {
//         if (!Dribble_flag)
//         {
//             MyGO[0].GO_Movement(50, 0.15, DRIBBLE_PRE_GO1_POSITION, 3, 0);
//             MyGO[1].GO_Movement(80, 0.15, DRIBBLE_PRE_GO2_POSITION, -3, 0);
//             MyGO[2].GO_Movement(5, 0.1, Dribble_Current_Time / float(Dribble_Total_Time) * DRIBBLE_GO3_POSITION, 3, 0);
//             // MyGO[2].GO_Velocity(8.5 + Dribble_Current_Time / float(Dribble_Total_Time) * 5.1, 0.1);
//             // MyGO[2].GO_Velocity(8.5 + Dribble_Current_Time / float(Dribble_Total_Time) * 5.5, 0.1);
//             Dribble_Current_Time++;
//             if (Dribble_Current_Time == Dribble_Total_Time)
//             {
//                 // MyGO[2].GO_position_speed(5.0,0.2,DRIBBLE_GO3_POSITION / 3,20.0,0.0);
//                 // MyGO[2].GO_Position(DRIBBLE_GO3_POSITION / 3, 0.5, 0.01);
//                 // MyGO[2].GO_Position(DRIBBLE_GO3_POSITION*0.8, 0.5, 0.01);
//                 Dribble_flag = 1;
//                 Shoot_Timer.delay_ms(300);
//                 Dribble_Receive_Ball();
//                 Shoot_Timer.delay_ms(1000);
//                 std::cout << "dribble finish" << std::endl;
//             }
//             Shoot_Timer.delay_ms(1.5);
//         }
//     }
// }

// // 运球复位
// void MANIPULATOR::Dribble_Reset_Position()
// {
//     if (!Dribble_Reset_flag)
//     {
//         if (Dribble_Reset_Current_Time < Dribble_Reset_Total_Time / 3)
//         {
//             MyGO[1].GO_Movement(5.0, 0.1, (Dribble_Reset_Current_Time) / float(Dribble_Reset_Total_Time / 3) * (-0.08), 0.0, 0);
//         }
//         else
//         {
//             MyGO[0].GO_Movement(5.0, 0.1, (Dribble_Reset_Current_Time - Dribble_Reset_Total_Time / 3) / float(Dribble_Reset_Total_Time * 2 / 3) * DRIBBLE_PRE_GO1_POSITION, 0.0, 0);
//             MyGO[1].GO_Movement(5.0, 0.1, (Dribble_Reset_Current_Time - Dribble_Reset_Total_Time / 3) / float(Dribble_Reset_Total_Time * 2 / 3) * (DRIBBLE_PRE_GO2_POSITION + 0.08) - 0.08, 0.0, 0);
//             MyGO[2].GO_Position((Dribble_Reset_Current_Time - Dribble_Reset_Total_Time / 3) / float(Dribble_Reset_Total_Time * 2 / 3) * (DRIBBLE_GO3_POSITION), 0.5, 0.01);
//             // MyGO[2].GO_Position((Dribble_Reset_Current_Time - Dribble_Reset_Total_Time / 3) / float(Dribble_Reset_Total_Time * 2 / 3) * (DRIBBLE_GO3_POSITION*0.8), 0.5, 0.01);
//         }
//         Dribble_Reset_Current_Time--;
//         if (Dribble_Reset_Current_Time == 0)
//         {
//             Dribble_Reset_flag = 1;
//             std::cout << "dribble reset finish" << std::endl;
//         }
//         Shoot_Timer.delay_ms(5);
//     }
// }

// 清空运球标志位和变量
void MANIPULATOR::Dribble_Clear_Flag()
{
    Dribble_Pre_Current_Time = 0;
    Dribble_Current_Time = 0;
    Dribble_Reset_Current_Time = Dribble_Reset_Total_Time;
    Dribble_Pre_flag = 0;
    Dribble_flag = 0;
    Dribble_Reset_flag = 0;
}

// 投篮准备动作
void MANIPULATOR::Shoot_Preparation(int time)
{
    // 机械臂初始动作，大臂微抬，手腕抬起
    if (!Shoot_Preparation_Change_flag)
    {
        MyGO[0].GO_Movement(5.0, 0.1, Shoot_Preparation_Current_time / float(Shoot_Preparation_Linear_total_time) * GO1_POSITION, 2, 0);
        MyGO[1].GO_Movement(5.0, 0.15, Shoot_Preparation_Current_time / float(Shoot_Preparation_Linear_total_time) * GO2_POSITION, -2, 0);
        if (Shoot_Preparation_Current_time > Shoot_Preparation_Linear_total_time / 2)
        {
            MyGO[2].GO_Movement(5.0, 0.05, (Shoot_Preparation_Current_time - Shoot_Preparation_Linear_total_time / 2) / float(Shoot_Preparation_Linear_total_time / 2) * GO3_POSITION, 2, 0.0);
        }
        Shoot_Preparation_Current_time++;
        Shoot_Timer.delay_ms(5);
        if (Shoot_Preparation_Current_time == Shoot_Preparation_Linear_total_time)
        {
            Shoot_Preparation_Change_flag = 1;
            Shoot_Timer.delay_ms(time);
            std::cout << "shoot preparation finish" << std::endl;
        }
    }
}

// 清空投篮准备动作标志位
void MANIPULATOR::Shoot_Preparation_Clear_Flag()
{
    Shoot_Preparation_Change_flag = 0;
    Shoot_Preparation_Current_time = 0;
}

// 投篮
void MANIPULATOR::Shoot_Basketball(int area, float distance)
{
    // 确定是哪一个投篮区域
    if (!Shoot_Choose_Area_flag && Shoot_Preparation_Change_flag)
    {
        if (distance < 0)
            distance = 0;
        else if (distance > THREE_POINT_LINE2CENTER_LINE_DISTANCE)
            distance = THREE_POINT_LINE2CENTER_LINE_DISTANCE;
        if (distance >= 0 && distance < BASKET2FREE_THROW_LINE_DISTANCE)
        {
            Shoot_Area = Basket2Free_Throw_Line;
            // Shoot_Distance = round(distance / POINT_LENGTH);
        }
        else if (distance < FREE_THROW_LINE2THREE_POINT_LINE_DISTANCE)
        {
            Shoot_Area = Free_Throw_Line2Three_Point_Line;
            // Shoot_Distance = round((distance - BASKET2FREE_THROW_LINE_DISTANCE) / POINT_LENGTH);
        }
        else if (distance <= THREE_POINT_LINE2CENTER_LINE_DISTANCE)
        {
            Shoot_Area = Three_Point_Line2Center_Line;
            // Shoot_Distance = round((distance - FREE_THROW_LINE2THREE_POINT_LINE_DISTANCE) / POINT_LENGTH);
        }
        // switch (area)
        // {
        // case 1:
        //     Shoot_Area = Basket2Free_Throw_Line;
        //     break;
        // case 2:
        //     Shoot_Area = Free_Throw_Line2Three_Point_Line;
        //     break;
        // case 3:
        //     Shoot_Area = Three_Point_Line2Center_Line;
        //     break;
        // default:
        //     break;
        // }
        Shoot_Distance = int(0.0);
        Shoot_Choose_Area_flag = 1;
        std::cout << "shoot choose area finish" << std::endl;
    }
    // 投篮执行过程
    if (!Shoot_flag && Shoot_Choose_Area_flag)
    {
        // 远投，先动小臂，再动大臂，再动手腕
        if (Shoot_Area == Three_Point_Line2Center_Line)
        {
            MyGO[1].GO_shoot(50, 0.2, Shoot_Area[Shoot_Distance].under_pos, Shoot_Area[Shoot_Distance].under_vel, Shoot_Area[Shoot_Distance].wrist_move_time, 1);
            if (fabs(MyGO[1].Pos_now) > fabs(Shoot_Area[Shoot_Distance].upper_move_time))
            {
                MyGO[0].GO_shoot(50, 0.2, Shoot_Area[Shoot_Distance].upper_pos, Shoot_Area[Shoot_Distance].upper_vel, 0.0, 1);
            }
            if (MyGO[1].delay_flag)
            {
                MyGO[2].GO_Movement(24, 0.1, Shoot_Area[Shoot_Distance].wrist_pos, Shoot_Area[Shoot_Distance].wrist_vel, 2.0); // 负  10  0.1
            }
        }
        // 近投，先动大臂，再动小臂，再动手腕
        else
        {
            MyGO[0].GO_shoot(50, 0.2, Shoot_Area[Shoot_Distance].upper_pos, Shoot_Area[Shoot_Distance].upper_vel, 0.0);
            if (fabs(MyGO[0].Pos_now) > fabs(Shoot_Area[Shoot_Distance].upper_move_time))
            {
                MyGO[1].GO_shoot(50, 0.2, Shoot_Area[Shoot_Distance].under_pos, Shoot_Area[Shoot_Distance].under_vel, Shoot_Area[Shoot_Distance].wrist_move_time);
            }
            if (MyGO[1].delay_flag)
            {
                MyGO[2].GO_Movement(10, 0.05, Shoot_Area[Shoot_Distance].wrist_pos, Shoot_Area[Shoot_Distance].wrist_vel, 2.0); // 负  10  0.1
            }
        }

        std::cout << "1111" << Shoot_Area[Shoot_Distance].upper_vel << "   Shoot_Distance" << Shoot_Distance << std::endl;

        Shoot_Timer.delay_ms(2);

        if (MyGO[0].GO_flag && MyGO[1].GO_flag)
        {
            // 投篮完成之后等待稳定
            if ((!Shoot_flag) && (!Shoot_Wait_flag))
            {
                Shoot_Wait_Current_time++;
                Shoot_Timer.delay_ms(1);
                if (Shoot_Wait_Current_time == Shoot_Area[Shoot_Distance].wait_time)
                {
                    Shoot_Wait_flag = 1;
                    Shoot_flag = 1;
                    Shoot_Preparation_Clear_Flag();
                    std::cout << "shoot finish" << std::endl;
                }
            }
        }
    }
}

// 机械臂恢复原位
void MANIPULATOR::Shoot_Reset_Position()
{
    // 机械臂缓慢恢复原位
    if ((!Shoot_Reset_flag) && Shoot_Wait_flag)
    {
        if (Shoot_Reset_Current_Time < Shoot_Reset_Total_Time / 5)
        {
            MyGO[1].GO_Movement(5.0, 0.1, Shoot_Reset_Current_Time / float(Shoot_Reset_Total_Time / 5) * (-0.08), 0.0, 0);
        }
        else
        {
            MyGO[0].GO_Movement(5.0, 0.1, (Shoot_Reset_Current_Time - Shoot_Reset_Total_Time / 5) / float(Shoot_Reset_Total_Time * 4 / 5) * (Shoot_Area[Shoot_Distance].upper_pos), 0.0, 0);
            MyGO[1].GO_Movement(5.0, 0.1, (Shoot_Reset_Current_Time - Shoot_Reset_Total_Time / 5) / float(Shoot_Reset_Total_Time * 4 / 5) * (Shoot_Area[Shoot_Distance].under_pos + 0.08) - 0.08, 0.0, 0);
            MyGO[2].GO_Position((Shoot_Reset_Current_Time - Shoot_Reset_Total_Time / 5) / float(Shoot_Reset_Total_Time * 4 / 5) * Shoot_Area[Shoot_Distance].wrist_pos, 0.5, 0.01);
        }
        Shoot_Reset_Current_Time--;
        if (Shoot_Reset_Current_Time == 0)
        {
            Shoot_Reset_flag = 1;
            std::cout << "shoot reset finish" << std::endl;
        }
        Shoot_Timer.delay_ms(5);
    }
}

// 清空投篮标志位和变量
void MANIPULATOR::Shoot_Clear_Flag()
{
    MyGO[0].GO_flag = 0;
    MyGO[0].delay_flag = 0;
    MyGO[1].GO_flag = 0;
    MyGO[1].delay_flag = 0;
    Shoot_flag = 0;
    Shoot_Choose_Area_flag = 0;
    Shoot_Reset_flag = 0;
    Shoot_Wait_Current_time = 0;
    Shoot_Wait_flag = 0;
    Shoot_Reset_Current_Time = Shoot_Reset_Total_Time;
}

// 传球
void MANIPULATOR::Pass_Ball(int area, float distance)
{
    Shoot_Preparation(2000);
    // 确定是哪一个投篮区域
    if (!Shoot_Choose_Area_flag && Shoot_Preparation_Change_flag)
    {
        Shoot_Area = Pass_All_Area;
        Shoot_Distance = int(0.0);
        Shoot_Choose_Area_flag = 1;
        std::cout << "Pass choose distance finish" << std::endl;
    }
    // 投篮执行过程
    if (!Shoot_flag && Shoot_Choose_Area_flag)
    {
        MyGO[1].GO_shoot(50, 0.2, Shoot_Area[Shoot_Distance].under_pos, Shoot_Area[Shoot_Distance].under_vel, Shoot_Area[Shoot_Distance].wrist_move_time, 1);
        if (fabs(MyGO[1].Pos_now) > fabs(Shoot_Area[Shoot_Distance].upper_move_time))
        {
            MyGO[0].GO_shoot(50, 0.2, Shoot_Area[Shoot_Distance].upper_pos, Shoot_Area[Shoot_Distance].upper_vel, 0.0, 1);
        }
        if (MyGO[1].delay_flag)
        {
            MyGO[2].GO_Movement(30, 0.1, Shoot_Area[Shoot_Distance].wrist_pos, Shoot_Area[Shoot_Distance].wrist_vel, 2.0); // 负  10  0.1
        }
        Shoot_Timer.delay_ms(2);

        if (MyGO[0].GO_flag && MyGO[1].GO_flag)
        {
            // 投篮完成之后等待稳定
            if ((!Shoot_flag) && (!Shoot_Wait_flag))
            {
                Shoot_Wait_Current_time++;
                Shoot_Timer.delay_ms(1);
                if (Shoot_Wait_Current_time == Shoot_Area[Shoot_Distance].wait_time)
                {
                    Shoot_Wait_flag = 1;
                    Shoot_flag = 1;
                    Shoot_Preparation_Clear_Flag();
                    std::cout << "pass finish" << std::endl;
                }
            }
        }
    }
}

void MANIPULATOR::Pass_Ball_Reset_Flag()
{
    Pass_Choose_Area_flag = 0;
    Pass_flag = 0;
    Pass_Reset_Flag = 0;
    Pass_Reset_Current_Time = Pass_Reset_Total_Time;
}

// 运球的接球
void MANIPULATOR::Dribble_Receive_Ball()
{
    mycan.cybergear[0]->motor_position(8 * ((1.62 - fabs(mycan.cybergear[0]->Current_pos)) / 1.62) + 5, -1.62);
    Shoot_Timer.delay_us(3000);
    mycan.cybergear[1]->motor_position(8 * ((1.62 - fabs(mycan.cybergear[0]->Current_pos)) / 1.62) + 5, 1.62);
    Shoot_Timer.delay_us(3000);
}

// 运球接完球后送到手里且复位
void MANIPULATOR::Trans2Paw()
{
    if (fabs(mycan.cybergear[0]->Current_pos) > 3.1 || Left_Dribble_Receive_Hand_Finish_Flag)
    {
        Left_Dribble_Receive_Hand_Finish_Flag = 1;
    }
    else
    {
        mycan.cybergear[0]->motor_position(7.5, -3.14);
        Shoot_Timer.delay_us(1000);
    }
    if (fabs(mycan.cybergear[1]->Current_pos) > 3.1 || Right_Dribble_Receive_Hand_Finish_Flag)
    {
        Right_Dribble_Receive_Hand_Finish_Flag = 1;
    }
    else
    {
        mycan.cybergear[1]->motor_position(7.5, 3.14);
        Shoot_Timer.delay_us(1000);
    }
    if (Left_Dribble_Receive_Hand_Finish_Flag && Right_Dribble_Receive_Hand_Finish_Flag && (!Trans2Paw_Flag))
    {
        mycan.cybergear[0]->motor_position(12.5, -3.85);
        Shoot_Timer.delay_us(1000);
        mycan.cybergear[1]->motor_position(12.5, 3.55);
        Shoot_Timer.delay_us(1000);
    }
    if ((fabs(mycan.cybergear[0]->Current_pos) > 3.8 && fabs(mycan.cybergear[1]->Current_pos) < 3.6) || Trans2Paw_Flag)
    {
        Trans2Paw_Flag = 1;
        Shoot_Timer.delay_ms(750);
        std::cout << "Trans2Paw finish" << std::endl;
    }
    if (Trans2Paw_Flag)
    {
        // if ((fabs(mycan.cybergear[0]->Current_pos) < 0.05) && (fabs(mycan.cybergear[1]->Current_pos) < 0.05))
        // {
        //     Trans2Paw_Reset_Flag = 1;
        //     std::cout << "Trans2Paw reset finish" << std::endl;
        // }
        // else
        // {
        mycan.cybergear[0]->motor_position(5.0, 0.0);
        Shoot_Timer.delay_us(1000);
        mycan.cybergear[1]->motor_position(5.0, 0.0);
        Shoot_Timer.delay_us(1000);
        std::cout << "Trans2Paw finish" << std::endl;
        // }
    }
}

void MANIPULATOR::Trans2Paw_Clear_Flag()
{
    Left_Dribble_Receive_Hand_Finish_Flag = 0;
    Right_Dribble_Receive_Hand_Finish_Flag = 0;
    Trans2Paw_Flag = 0;
    Trans2Paw_Reset_Flag = 0;
}

// 张开爪子
void MANIPULATOR::Open_Paw()
{
    // 张开爪子
    if (!Open_Paw_Flag)
    {
        MyGO[0].GO_Movement(5.0, 0.15, Open_Paw_Current_Time / float(Open_Paw_Total_Time) * Open_Paw_Go1_Pos, 0, 0);
        MyGO[1].GO_Movement(2.0, 0.05, Open_Paw_Current_Time / float(Open_Paw_Total_Time) * Open_Paw_Go2_Pos, 2, 1.5);
        MyGO[2].GO_Position(Open_Paw_Current_Time / float(Open_Paw_Total_Time) * Open_Paw_Go3_Pos, 0.5, 0.01);
        Shoot_Timer.delay_ms(3);
        Open_Paw_Current_Time++;
        if (Open_Paw_Current_Time == Open_Paw_Total_Time)
            Open_Paw_Flag = 1;
    }
    // // 等待2s
    // else
    // {
    //     Shoot_Timer.delay_ms(200);
    //     Open_Paw_Finish_Wait_Current_Time++;
    //     std::cout << "Open_Paw_Current_Time" << Open_Paw_Finish_Wait_Current_Time << "Open_Paw_Finish_Wait_Flag" << Open_Paw_Finish_Wait_Flag << std::endl;
    //     if (Open_Paw_Finish_Wait_Current_Time == Open_Paw_Finish_Wait_Time)
    //     {
    //         Open_Paw_Finish_Wait_Flag = 1;
    //     }
    // }
}

void MANIPULATOR::Open_Paw_Clear_Flag()
{
    Open_Paw_Flag = 0;
    Open_Paw_Current_Time = 0;
    Open_Paw_Finish_Wait_Flag = 0;
}

// 爪子复位
void MANIPULATOR::Close_Paw()
{
    if (!Close_Paw_Flag)
    {
        if (Close_Paw_Current_Time >= Close_Paw_Total_Time / 2)
        {
            MyGO[0].GO_Movement(5.0, 0.1, (Close_Paw_Current_Time - Close_Paw_Total_Time / 2) / float(Close_Paw_Total_Time / 2) * Open_Paw_Go1_Pos, 0.0, 0);
            MyGO[2].GO_Position((Close_Paw_Current_Time - Close_Paw_Total_Time / 2) / float(Close_Paw_Total_Time / 2) * Open_Paw_Go3_Pos, 0.5, 0.01);
        }
        else
        {
            MyGO[1].GO_Movement(5.0, 0.1, Close_Paw_Current_Time / float(Close_Paw_Total_Time / 2) * Open_Paw_Go2_Pos, 0.0, 0);
        }
        Shoot_Timer.delay_ms(3);
        Close_Paw_Current_Time--;
        if (Close_Paw_Current_Time == 0)
            Close_Paw_Flag = 1;
    }
}

void MANIPULATOR::Close_Paw_Clear_Flag()
{
    Close_Paw_Flag = 0;
    Close_Paw_Current_Time = Close_Paw_Total_Time;
}

// 传球的接球
void MANIPULATOR::Pass_Receive_Ball()
{
    MyGO[3].GO_Movement(0.5, 0.01, Common_Receive_Ball_Pos + Pass_Receive_Ball_Current_time / float(Pass_Receive_Ball_Total_time) * (Pass_Receive_Ball_Pos - Common_Receive_Ball_Pos), 0.1, 0);
    // MyGO[3].GO_Movement(2.5, 0.05, Common_Receive_Ball_Pos + Pass_Receive_Ball_Current_time / float(Pass_Receive_Ball_Total_time) * (Pass_Receive_Ball_Pos - Common_Receive_Ball_Pos), 0.1, 0);
    Open_Paw();
    if (Pass_Receive_Ball_Current_time < Pass_Receive_Ball_Total_time)
    {
        Pass_Receive_Ball_Current_time++;
    }
    else
    {
        Pass_Receive_Ball_Finish_Flag = 1;
    }
}

void MANIPULATOR::Pass_Receive_Ball_Clear_Flag()
{
    Pass_Receive_Ball_Finish_Flag = 0;
    Pass_Receive_Ball_Current_time = 0;
}

// 传球的接球复位
void MANIPULATOR::Pass_Receive_Ball_Reset()
{
    MyGO[3].GO_Movement(2.5, 0.05, Common_Receive_Ball_Pos + Pass_Receive_Ball_Reset_Current_time / float(Pass_Receive_Ball_Reset_Total_time) * (Pass_Receive_Ball_Pos - Common_Receive_Ball_Pos), 0.0, 0);
    Close_Paw();
    if (Pass_Receive_Ball_Reset_Current_time > 0)
    {
        Pass_Receive_Ball_Reset_Current_time--;
    }
    else
    {
        Pass_Receive_Ball_Reset_Finish_Flag = 1;
    }
}

void MANIPULATOR::Pass_Receive_Ball_Reset_Clear_Flag()
{
    Pass_Receive_Ball_Reset_Finish_Flag = 0;
    Pass_Receive_Ball_Reset_Current_time = Pass_Receive_Ball_Reset_Total_time;
}

// 终止当前动作且复位
void MANIPULATOR::Cancel_Reset_Position()
{
    if (!Cancel_Reset_Position_Pre_Flag)
    {
        for (int i = 0; i < 4; i++)
        {
            Reset_Position_Go_Pos[i] = MyGO[i].Pos_now;
        }
        for (int i = 0; i < 2; i++)
        {
            Reset_Position_CyberGear_Pos[i] = mycan.cybergear[i]->Current_pos;
        }
        Cancel_Reset_Position_Pre_Flag = 1;
    }

    if (Cancel_Reset_Position_Current_time < Cancel_Reset_Position_Total_time / 4)
    {
        MyGO[1].GO_Movement(5.0, 0.1, (Cancel_Reset_Position_Current_time) / float(Cancel_Reset_Position_Total_time / 4) * (-0.08), 0.0, 0);
    }
    else
    {
        MyGO[0].GO_Movement(5.0, 0.1, (Cancel_Reset_Position_Current_time - Cancel_Reset_Position_Total_time / 4) / float(Dribble_Reset_Total_Time * 3 / 4) * (Reset_Position_Go_Pos[0]), 0.0, 0);
        MyGO[1].GO_Movement(5.0, 0.1, (Cancel_Reset_Position_Current_time - Cancel_Reset_Position_Total_time / 4) / float(Dribble_Reset_Total_Time * 3 / 4) * (Reset_Position_Go_Pos[1] + 0.08) - 0.08, 0.0, 0);
        MyGO[2].GO_Position((Cancel_Reset_Position_Current_time - Cancel_Reset_Position_Total_time / 4) / float(Dribble_Reset_Total_Time * 3 / 4) * (Reset_Position_Go_Pos[2]), 0.5, 0.01);
    }

    // if (fabs(MyGO[1].Pos_now) < 0.08)
    // {
    //     MyGO[0].GO_Movement(5.0, 0.1, 0.0, 0.0, 0);
    //     MyGO[1].GO_Movement(5.0, 0.1, Cancel_Reset_Position_Current_time / float(Cancel_Reset_Position_Total_time) * (Reset_Position_Go_Pos[1]), 0.0, 0);
    //     MyGO[2].GO_Position(0.0, 0.5, 0.01);
    // }
    // else
    // {
    //     MyGO[0].GO_Movement(5.0, 0.1, Cancel_Reset_Position_Current_time / float(Cancel_Reset_Position_Total_time) * (Reset_Position_Go_Pos[0]), 0.0, 0);
    //     MyGO[1].GO_Movement(5.0, 0.1, Cancel_Reset_Position_Current_time / float(Cancel_Reset_Position_Total_time) * (Reset_Position_Go_Pos[1]), 0.0, 0);
    //     MyGO[2].GO_Position(Cancel_Reset_Position_Current_time / float(Cancel_Reset_Position_Total_time) * (Reset_Position_Go_Pos[2]), 0.5, 0.01);
    // }

    // if (Cancel_Reset_Position_Current_time > Cancel_Reset_Position_Total_time / 2)
    // {
    //     MyGO[2].GO_Position(Reset_Position_Go_Pos[2], 0.5, 0.01);
    // }
    // else
    // {
    //     MyGO[2].GO_Position(Cancel_Reset_Position_Current_time / float(Cancel_Reset_Position_Total_time / 2) * (Reset_Position_Go_Pos[2]), 0.5, 0.01);
    // }
    MyGO[3].GO_Movement(2.5, 0.05, Common_Receive_Ball_Pos + Cancel_Reset_Position_Current_time / float(Cancel_Reset_Position_Total_time) * (Reset_Position_Go_Pos[3] - Common_Receive_Ball_Pos), 0.0, 0); // 负
    mycan.cybergear[0]->motor_position(5.0, Cancel_Reset_Position_Current_time / float(Cancel_Reset_Position_Total_time) * (Reset_Position_CyberGear_Pos[0]));
    mycan.cybergear[1]->motor_position(5.0, Cancel_Reset_Position_Current_time / float(Cancel_Reset_Position_Total_time) * (Reset_Position_CyberGear_Pos[1]));

    Shoot_Timer.delay_ms(5);
    if (Cancel_Reset_Position_Current_time > 0)
    {
        Cancel_Reset_Position_Current_time--;
    }
    else
    {
        Cancel_Reset_Position_Finish_Flag = 1;
    }
}

// 清空标志位
void MANIPULATOR::Cancel_Reset_Position_Clear_Flag()
{
    Cancel_Reset_Position_Current_time = Cancel_Reset_Position_Total_time;
    Cancel_Reset_Position_Pre_Flag = 0;
    Cancel_Reset_Position_Finish_Flag = 0;
}

// 停止动作时保持在当前位置，投篮过程不建议用，未测
void MANIPULATOR::Stay_Current_position()
{
    if (!Stay_Position_Pre_Flag)
    {
        for (int i = 0; i < 4; i++)
        {
            Stay_Position_Go_Pos[i] = MyGO[i].Pos_now;
        }
        for (int i = 0; i < 2; i++)
        {
            Stay_Position_CyberGear_Pos[i] = mycan.cybergear[i]->Current_pos;
        }
        Stay_Position_Pre_Flag = 1;
    }

    MyGO[0].GO_Movement(5.0, 0.1, Stay_Position_Go_Pos[0], 0.0, 0);
    Shoot_Timer.delay_ms(0.5);
    MyGO[1].GO_Movement(5.0, 0.1, Stay_Position_Go_Pos[1], 0.0, 0);
    Shoot_Timer.delay_ms(0.5);
    MyGO[2].GO_Position(Stay_Position_Go_Pos[2], 0.5, 0.01);
    Shoot_Timer.delay_ms(0.5);
    MyGO[3].GO_Movement(2.5, 0.05, Stay_Position_Go_Pos[3], 0.0, 0); // 负
    Shoot_Timer.delay_ms(0.5);
    mycan.cybergear[0]->motor_position(5.0, Stay_Position_CyberGear_Pos[0]);
    Shoot_Timer.delay_ms(0.5);
    mycan.cybergear[1]->motor_position(5.0, Stay_Position_CyberGear_Pos[1]);
    Shoot_Timer.delay_ms(0.5);
}

// 测试用
void MANIPULATOR::test()
{
    mycan.cybergear[0]->motor_position(5.0, 0.0);
    Shoot_Timer.delay_us(1000);
    mycan.cybergear[1]->motor_position(5.0, 0.0);
    Shoot_Timer.delay_us(1000);
}

void MANIPULATOR::Empty() {}
