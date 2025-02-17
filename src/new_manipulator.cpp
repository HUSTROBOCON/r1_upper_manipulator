#include "new_manipulator.hpp"
#include "Motor/Cybergear/can.hpp"
#include "Public_function.hpp"
#include "Shoot.hpp"

MANIPULATOR myshoot;
MYTIMER manipulator_timer;

CurryNode::CurryNode() : Node("curry_node"),
                         LastState(0),
                         CurrentState(0),
                         Shoot_Distance(3.0),
                         stop_shoot(true),
                         is_Clear_Flag(false)
{
    curry_server = rclcpp_action::create_server<Curry>(
        this,
        "curry",
        [this](const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Curry::Goal> goal) -> rclcpp_action::GoalResponse
        {
            return this->curryHandleGoal(uuid, goal);
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Curry>> curry_handle) -> rclcpp_action::CancelResponse
        {
            return this->curryCancel(curry_handle);
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Curry>> curry_handle)
        {
            this->curryExecute(curry_handle);
        });
}

rclcpp_action::GoalResponse CurryNode::curryHandleGoal(const rclcpp_action::GoalUUID &uuid,
                                                       std::shared_ptr<const Curry::Goal> goal)
{
    (void)uuid;
    LastState = CurrentState;
    CurrentState = goal->control_mode;
    Shoot_Distance = goal->distance;
    RCLCPP_WARN(this->get_logger(), "control_mode : %d", CurrentState);
    if (stop_shoot)
    {
        stop_shoot = false;
        is_Clear_Flag = true;
        RCLCPP_WARN(this->get_logger(), "New Control, execute!");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    else
    {
        // 动作未执行完成，用已存在线程继续执行，防止重复创建多个线程
        RCLCPP_WARN(this->get_logger(), "already Curry, reject!");
        return rclcpp_action::GoalResponse::REJECT;
    }
};
rclcpp_action::CancelResponse CurryNode::curryCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Curry>> curry_handle)
{
    (void)curry_handle;
    stop_shoot = true;
    RCLCPP_INFO(this->get_logger(), "Curry Cancel, Reset Position");
    return rclcpp_action::CancelResponse::ACCEPT;
};
void CurryNode::curryExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Curry>> curry_handle)
{
    RCLCPP_INFO(this->get_logger(), "CurryExecute");
    std::thread([this, curry_handle]()
                {
    rclcpp::Rate loop_rate(1000);
    auto feedback = std::make_shared<Curry::Feedback>();
    const auto goal =  curry_handle->get_goal(); 
    while(!stop_shoot){
        rclcpp::spin_some(mycan.cybergear[0]->node);
        rclcpp::spin_some(mycan.cybergear[1]->node);
        switch (CurrentState)
        {
            case SHOOT_PREPARATION:
                if (is_Clear_Flag)
                {
                    myshoot.Shoot_Preparation_Clear_Flag();
                    is_Clear_Flag = false;
                }
                myshoot.Shoot_Preparation(2000);
                if(myshoot.Shoot_Preparation_Change_flag) 
                    stop_shoot = true;
                break;
            case SHOOT:
                if (is_Clear_Flag)
                {
                    myshoot.Shoot_Clear_Flag();
                    is_Clear_Flag = false;

                }
                if (!myshoot.Shoot_flag)
                {
                    myshoot.Shoot_Basketball(FREE_THROW_LINE2THREE_POINT_LINE, Shoot_Distance);
                    // myshoot.Shoot_Basketball(FREE_THROW_LINE2THREE_POINT_LINE, 3.7);

                }
                else
                {
                    myshoot.Shoot_Reset_Position();
                }
                if(myshoot.Shoot_Reset_flag) stop_shoot = true;
                break;
            case DRIBBLE:
                if (is_Clear_Flag)
                {
                    myshoot.Dribble_Clear_Flag();
                    myshoot.Trans2Paw_Clear_Flag();
                    is_Clear_Flag = false;
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
                if(myshoot.Trans2Paw_Flag) stop_shoot = true;
                break;
            case PASS_RECEIVE_BALL:
                if (is_Clear_Flag)
                {
                    myshoot.Open_Paw_Clear_Flag();
                    myshoot.Pass_Receive_Ball_Clear_Flag();
                    is_Clear_Flag = false;
                }
                if (!myshoot.Pass_Receive_Ball_Finish_Flag)
                {
                    myshoot.Pass_Receive_Ball();
                }
                if(myshoot.Pass_Receive_Ball_Finish_Flag) stop_shoot = true;
                break;
            case PASS_BALL:
                if (is_Clear_Flag)
                {
                    myshoot.Shoot_Clear_Flag();
                    myshoot.Shoot_Preparation_Clear_Flag();
                    is_Clear_Flag = false;

                }
                if (!myshoot.Shoot_flag)
                {
                    myshoot.Pass_Ball(1, 0.0);
                }
                else
                {
                    myshoot.Shoot_Reset_Position();
                }
                if(myshoot.Shoot_Reset_flag) stop_shoot = true;
                break;
                // if (is_Clear_Flag)
                // {
                //     myshoot.Pass_Ball_Reset_Flag();
                //     is_Clear_Flag = false;
                // }
                // if (!myshoot.Pass_Reset_Flag)
                // {
                //     myshoot.Pass_Ball(1,0.0);
                // }
                // if(myshoot.Pass_Reset_Flag) stop_shoot = true;
                // break;
            case PASS_RECEIVE_BALL_RESET:
                if (is_Clear_Flag)
                {
                    myshoot.Close_Paw_Clear_Flag();
                    myshoot.Pass_Receive_Ball_Reset_Clear_Flag();
                    is_Clear_Flag = false;
                }
                if(LastState!=PASS_RECEIVE_BALL){
                    stop_shoot = true;
                }
                else{
                    if (!myshoot.Pass_Receive_Ball_Reset_Finish_Flag)
                    {
                        myshoot.Pass_Receive_Ball_Reset();
                    }
                    if(myshoot.Pass_Receive_Ball_Reset_Finish_Flag) stop_shoot = true;
                }
                break;
            case CANCEL_RESET_POSITION:
                if (is_Clear_Flag)
                {
                    myshoot.Cancel_Reset_Position_Clear_Flag();
                    is_Clear_Flag = false;
                }
                if (!myshoot.Cancel_Reset_Position_Finish_Flag)
                {
                    myshoot.Cancel_Reset_Position();
                }
                if(myshoot.Cancel_Reset_Position_Finish_Flag) stop_shoot = true;
                break;
            default:
                break;
        }
            loop_rate.sleep();
    }
    // myshoot.Stay_Position_Pre_Flag = 0;
    // myshoot.Stay_Current_position();
    auto result = std::make_shared<Curry::Result>();
     curry_handle->succeed(result); })
        .detach();
};

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    manipulator_timer.delay_sec(2);
    myshoot.Manipulator_Init();
    rclcpp::spin(std::make_shared<CurryNode>());
    rclcpp::shutdown();
    return 0;
}