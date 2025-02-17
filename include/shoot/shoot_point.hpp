#ifndef SHOOT_POINT_HPP
#define SHOOT_POINT_HPP

#include "Motor/Go/GO_base.hpp"

#define POINT_LENGTH 0.05

#define BASKET2FREE_THROW_LINE 1
#define FREE_THROW_LINE2THREE_POINT_LINE 2
#define THREE_POINT_LINE2CENTER_LINE 3

#define BASKET2FREE_THROW_LINE_DISTANCE 2.0
#define FREE_THROW_LINE2THREE_POINT_LINE_DISTANCE 3.1
#define THREE_POINT_LINE2CENTER_LINE_DISTANCE 6.0

#define Basket2Free_Throw_Line_Num 1
#define Free_Throw_Line2Three_Point_Line_Num 1
#define Three_Point_Line2Center_Line_Num 1
#define PASS_AREA_NUM 1

typedef struct
{
    float upper_pos;
    float upper_vel;
    float upper_move_time;
    float under_pos;
    float under_vel;
    float wrist_move_time;
    float wrist_pos;
    float wrist_vel;
    int wait_time;
} SHOOT_POINT;

// 调参：小臂速度至少为大臂两倍或更多  小臂速度越小越近
// 手腕出手时机越晚弧线越低
// 手腕初始位置压的越低越容易拨腕
// 手腕只需要改出手时机和行程，该其他参数效果不大 行程越大，速度越快
// 小臂大臂速度越快，球出射越快
// 大臂速度不能过快，否则会翻过区，大臂相对小臂速度越快，偏向上，越慢，偏向远

// 一个区间最后一个投篮点和下一个投篮区间第一个投篮点取相同点
static SHOOT_POINT Basket2Free_Throw_Line[Basket2Free_Throw_Line_Num]{
    // upper_pos        upper_vel   under_move_time under_pos under_vel wrist_move_time wrist_pos wrist_vel
    // {GO1_POSITION + 1.8, 6.0, GO1_POSITION, -2.5, -20, -1.2, 1.4, 25}};
    // {GO1_POSITION+2.15 , 5.5, GO1_POSITION, -2.7, -14.0, -0.8, 1.2, 7.5}};//1.4
    // {GO1_POSITION + 1.95, 8.0, -0.133, -2.7, -14.0, -2.05, 0.5, 10.0, 1000}}; //
    {GO1_POSITION + 1.85, 8.0, -0.133, -2.7, -14.0, -2.05, 0.5, 10.0, 350}}; // 超高弧线传球
//
static SHOOT_POINT Free_Throw_Line2Three_Point_Line[Free_Throw_Line2Three_Point_Line_Num]{
    // upper_pos upper_vel under_move_time under_pos under_vel wrist_move_time wrist_pos wrist_vel
    {GO1_POSITION + 1.8, 4.5, GO1_POSITION + 0.075, -2.7, -9.175, -1.4, 0.725, 7.5, 400}}; // 1.4

static SHOOT_POINT Three_Point_Line2Center_Line[Three_Point_Line2Center_Line_Num]{
    // upper_pos upper_vel under_move_time under_pos under_vel wrist_move_time wrist_pos wrist_vel
    // {GO1_POSITION + 1.8, 5.35, -0.1, -2.7, -12.5, -1.4, 0.95, 10.0,500}}; // 三分线 !!!等待时间为500！！！记得改
    // {GO1_POSITION + 1.8, 5.85, -0.1, -2.7, -13.25, -1.4, 0.885, 10.0, 500}}; // 1.4
    {GO1_POSITION + 1.9, 5.6, -0.1, -2.7, -13.625, -1.6, 0.7, 10.0, 500}}; // 1.4

// 5.35

static SHOOT_POINT Pass_All_Area[PASS_AREA_NUM]{
    // upper_pos        upper_vel   under_move_time under_pos under_vel wrist_move_time wrist_pos wrist_vel
    // {0.0, 3.0, 0.0, 0.0, -1.5, 0.0, 1.3, 15.0, 350}}; //
    {GO1_POSITION + 1.8, 4.5, -0.1, -2.7, -8.0, -1.8, 0.9, 5.0, 500}}; // 1.4
#endif
