
#include "system_controller/SystemController.hpp"
#include <algorithm>

constexpr double WarningSpeed = 0.1;

using std::placeholders::_1;

SystemController::SystemController(const std::string &name): rclcpp_lifecycle::LifecycleNode(name)
{
    this->_cmdVelPub = this->create_publisher<geometry_msgs::msg::Twist>("/roomba/cmd_vel", 10);

    this->_cmdVelSub = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&SystemController::cmdVelCallback, this, _1));

    this->_bumperSub = this->create_subscription<create_msgs::msg::Bumper>("/roomba/bumper", 10, std::bind(&SystemController::bumperCallback, this, _1));

    this->declare_parameter("status_timeout", 1.0);

    double statusTimeout;

    this->get_parameter("status_timeout", statusTimeout);
    this->_status.SetTimeout(rclcpp::Duration::from_seconds(statusTimeout));

}

void SystemController::bumperCallback(const create_msgs::msg::Bumper::ConstSharedPtr msg)
{
    auto status = this->_status.GetData();
    status.bumper = *msg;
    this->_status.SetData(status);
}

void SystemController::cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
    geometry_msgs::msg::Twist toSend;

    if(this->_status.IsDataTimeout())
    {
        this->_cmdVelPub->publish(toSend);
        return;
    }

    toSend = *msg;

    const auto status = this->_status.GetData();

    if(status.bumper.is_left_pressed || status.bumper.is_right_pressed)
    {
        toSend.linear.x = std::min(0.0, toSend.linear.x);
        this->_cmdVelPub->publish(toSend);
        return;
    }

    if(status.bumper.is_light_left || status.bumper.is_light_right)
    {
        toSend.linear.x = std::min(WarningSpeed, toSend.linear.x);
        this->_cmdVelPub->publish(toSend);
        return;
    }

    this->_cmdVelPub->publish(toSend);



}

