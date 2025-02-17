#ifndef SHOOT_HPP
#define SHOOT_HPP

#include "shoot_point.hpp"

extern CAN mycan;
extern GO_Drive MyGO[4];

class MANIPULATOR
{
private:
    int Dribble_Pre_Current_Time = 0;
    int Dribble_Pre_Total_Time = 150; // 116//225
    // int Dribble_Pre_Total_Time = 250; // 116//225
    int Dribble_Current_Time = 0;
    int Dribble_Total_Time = 60;    //12_30   40
    // int Dribble_Total_Time = 250;
    int Dribble_Reset_Total_Time = 251;
    int Dribble_Reset_Current_Time = Dribble_Reset_Total_Time;
    int Shoot_Preparation_Current_time = 0;
    int Shoot_Preparation_Linear_total_time = 50;
    int Shoot_Wait_Current_time = 0;
    int Shoot_Wait_total_time = 350; // 三分线
    int Shoot_Reset_Total_Time = 250;
    int Shoot_Reset_Current_Time = Shoot_Reset_Total_Time;
    int Pass_Reset_Total_Time = 250;
    int Pass_Reset_Current_Time = Pass_Reset_Total_Time;
    int Open_Paw_Current_Time = 0;
    int Open_Paw_Total_Time = 53;
    int Open_Paw_Finish_Wait_Current_Time = 0;
    int Open_Paw_Finish_Wait_Time = 10;
    int Close_Paw_Total_Time = 50;
    int Close_Paw_Current_Time = Close_Paw_Total_Time;
    int Pass_Receive_Ball_Current_time = 0;
    int Pass_Receive_Ball_Total_time = 200;
    int Pass_Receive_Ball_Reset_Total_time = 200;
    int Pass_Receive_Ball_Reset_Current_time = Pass_Receive_Ball_Reset_Total_time;
    int Cancel_Reset_Position_Current_time = 0;
    int Cancel_Reset_Position_Total_time = 300;

    float Reset_Position_Go_Pos[4] = {0.0};
    float Reset_Position_CyberGear_Pos[2] = {0.0};
    float Stay_Position_Go_Pos[4] = {0.0};
    float Stay_Position_CyberGear_Pos[2] = {0.0};

public:
    SHOOT_POINT *Shoot_Area;
    SHOOT_POINT *Pass_Area;
    int Shoot_Distance = 0;
    int Shoot_flag = 0;
    int Shoot_Wait_flag = 0;
    int Shoot_Reset_flag = 0;
    int Shoot_Preparation_Change_flag = 0;
    int Shoot_Choose_Area_flag = 0;

    int Pass_Distance = 0;
    int Pass_Choose_Area_flag = 0;
    int Pass_flag = 0;
    int Pass_Reset_Flag = 0;

    int Dribble_Pre_flag = 0;
    int Dribble_flag = 0;
    int Dribble_Reset_flag = 0;
    int Left_Dribble_Receive_Hand_Finish_Flag = 0;
    int Right_Dribble_Receive_Hand_Finish_Flag = 0;
    int Trans2Paw_Flag = 0;
    int Trans2Paw_Reset_Flag = 0;

    int Open_Paw_Flag = 0;
    int Close_Paw_Flag = 0;
    int Open_Paw_Finish_Wait_Flag = 0;
    int Pass_Receive_Ball_Finish_Flag = 0;
    int Pass_Receive_Ball_Reset_Finish_Flag = 0;

    int Cancel_Reset_Position_Pre_Flag = 0;
    int Cancel_Reset_Position_Finish_Flag = 0;
    int Stay_Position_Pre_Flag = 0;

    float Open_Paw_Go1_Pos = 0.3;   // 0.3
    float Open_Paw_Go2_Pos = 0.05;  //-5.0 / 57.3
    float Open_Paw_Go3_Pos = 0.075; //-0.05
    float Common_Receive_Ball_Pos = 1.05;
    float Pass_Receive_Ball_Pos = 1.5;

    char CurrentState = 0;
    char LastState = 0;

    void Manipulator_Init();
    void Read_position();
    void Shoot_Preparation(int time);
    void Shoot_Preparation_Clear_Flag();
    void Shoot_Basketball(int area, float distance);
    void Shoot_Reset_Position();
    void Shoot_Clear_Flag();
    void Pass_Ball(int area, float distance);
    void Pass_Ball_Reset_Flag();
    void Dribble();
    void Dribble_Reset_Position();
    void Dribble_Clear_Flag();
    void Dribble_Receive_Ball();
    void Open_Paw();
    void Open_Paw_Clear_Flag();
    void Close_Paw();
    void Close_Paw_Clear_Flag();
    void Trans2Paw();
    void Trans2Paw_Clear_Flag();
    void Pass_Receive_Ball();
    void Pass_Receive_Ball_Clear_Flag();
    void Pass_Receive_Ball_Reset_Clear_Flag();
    void Pass_Receive_Ball_Reset();
    void Cancel_Reset_Position();
    void Cancel_Reset_Position_Clear_Flag();
    void Stay_Current_position();
    void test();
    void Empty();
};

#endif