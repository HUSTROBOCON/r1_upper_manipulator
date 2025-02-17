#include "Motor/Cybergear/cybergear_base.hpp"
#include "Motor/Cybergear/can.hpp"
#include "Timer.hpp"
#include <unistd.h>
#include "Motor/Go/serialPort/SerialPort.h"
#include "Motor/Go/unitreeMotor/unitreeMotor.h"
#include "Motor/Go/GO_base.hpp"
#include "Shoot.hpp"
#include "rclcpp/rclcpp.hpp"
#include "/usr/include/boost/parameter.hpp"
#include "m3508_base.hpp"
#include "MotorPid.hpp"
#include "Public_function.hpp"

MYTIMER mytimer;
MANIPULATOR myshoot;
int8_t input_;

int main(int argc, char **argv)
{
  mytimer.delay_sec(0.5);
  rclcpp::init(argc, argv);
  rclcpp::Rate loop_rate(1000);
  myshoot.Manipulator_Init();
  std::cout << "Init Finished, input control to start" << endl;
  while (rclcpp::ok())
  {
    rclcpp::spin_some(mycan.m3508[0]->node);
    rclcpp::spin_some(mycan.m3508[1]->node);
    rclcpp::spin_some(mycan.cybergear[0]->node);
    rclcpp::spin_some(mycan.cybergear[1]->node);

    input_ = get_char();
    if (input_ != 0 && input_ != '\n')
    {
      myshoot.CurrentState = input_;
      std::cout << "myshoot.CurrentState   " << myshoot.CurrentState << std::endl;
    }
    switch (myshoot.CurrentState)
    {
    case SHOOT_PREPARATION:
      if (input_ == SHOOT_PREPARATION)
      {
        myshoot.Shoot_Preparation_Clear_Flag();
      }
      myshoot.Shoot_Preparation(2000);
      break;
    case SHOOT:
      if (input_ == SHOOT)
      {
        myshoot.Shoot_Clear_Flag();
      }
      if (!myshoot.Shoot_flag)
      {
        myshoot.Shoot_Basketball(BASKET2FREE_THROW_LINE, 0.0);
      }
      else
      {
        myshoot.Shoot_Reset_Position();
      }
      break;
    case DRIBBLE_RECEIVE_BALL:
      myshoot.Dribble_Receive_Ball();
      break;
    case TRANS2PAW:
      if (input_ == TRANS2PAW)
      {
        myshoot.Trans2Paw_Clear_Flag();
      }
      if (!myshoot.Trans2Paw_Flag)
      {
        myshoot.Trans2Paw();
      }
      break;
    case DRIBBLE:
      if (input_ == DRIBBLE)
      {
        myshoot.Dribble_Clear_Flag();
        myshoot.Trans2Paw_Clear_Flag();
      }
      if (!myshoot.Dribble_flag)
      {
        myshoot.Dribble();
      }
      else
      {
        myshoot.Dribble_Reset_Position();
      }
      if (myshoot.Dribble_Reset_flag && (!myshoot.Trans2Paw_Flag))
      {
        myshoot.Trans2Paw();
      }
      break;
    case OPEN_PAW:
      if (input_ == OPEN_PAW)
      {
        myshoot.Open_Paw_Clear_Flag();
        // myshoot.Close_Paw_Clear_Flag();
      }
      if (!myshoot.Open_Paw_Flag)
      {
        myshoot.Open_Paw();
      }
      // else
      // {
      //   myshoot.Close_Paw();
      // }
      break;
    case CLOSE_PAW:
      if (input_ == CLOSE_PAW)
      {
        myshoot.Close_Paw_Clear_Flag();
      }
      if (!myshoot.Close_Paw_Flag)
      {
        myshoot.Close_Paw();
      }
      break;
    case PASS_RECEIVE_BALL:
      if (input_ == PASS_RECEIVE_BALL)
      {
        myshoot.Open_Paw_Clear_Flag();
        myshoot.Pass_Receive_Ball_Clear_Flag();
      }
      if (!myshoot.Pass_Receive_Ball_Finish_Flag)
      {
        myshoot.Pass_Receive_Ball();
      }
      break;
    case PASS_RECEIVE_BALL_RESET:
      if (input_ == PASS_RECEIVE_BALL_RESET)
      {
        myshoot.Close_Paw_Clear_Flag();
        myshoot.Pass_Receive_Ball_Reset_Clear_Flag();
      }
      if (myshoot.Pass_Receive_Ball_Reset_Finish_Flag)
      {
        myshoot.Pass_Receive_Ball_Reset();
      }
      break;
    case CANCEL_RESET_POSITION:
      if (input_ == CANCEL_RESET_POSITION)
      {
        myshoot.Cancel_Reset_Position_Clear_Flag();
      }
      if (!myshoot.Cancel_Reset_Position_Finish_Flag)
      {
        myshoot.Cancel_Reset_Position();
      }
      break;
    case TEST:
      myshoot.test();
      break;
    case EMPTY:
      myshoot.Empty();
      break;
    default:
      break;
    }
    loop_rate.sleep();
  }
  return 0;
}