#ifndef NEW_MANIPULATOR_HPP
#define NEW_MANIPULATOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "comlib/action/curry.hpp"

using Curry = comlib::action::Curry;

class CurryNode : public rclcpp::Node
{

public:
    CurryNode();

private:
    // 导航服务器
    rclcpp_action::Server<Curry>::SharedPtr curry_server;
    rclcpp_action::GoalResponse curryHandleGoal(const rclcpp_action::GoalUUID &uuid,
                                                std::shared_ptr<const Curry::Goal> goal);
    rclcpp_action::CancelResponse curryCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Curry>> curry_handle);
    void curryExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Curry>> curry_handle);

    int8_t LastState;
    int8_t CurrentState;
    float Shoot_Distance;
    bool stop_shoot;
    bool is_Clear_Flag;
};
#endif