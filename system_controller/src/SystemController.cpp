#include "system_controller/SystemController.hpp"
#include <algorithm>

constexpr double WarningSpeed = 0.05;
constexpr uint16_t WarningIntensity = 100;

using std::placeholders::_1;

SystemController::SystemController(const std::string& name) : rclcpp::Node(name)
{
  this->_cmdVelPub = this->create_publisher<geometry_msgs::msg::Twist>("/roomba/cmd_vel", 10);

  this->_cmdVelSub = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&SystemController::cmdVelCallback, this, _1));

  this->_bumperSub = this->create_subscription<create_msgs::msg::Bumper>(
      "/roomba/bumper", 10, std::bind(&SystemController::bumperCallback, this, _1));

  this->_cliffSub = this->create_subscription<create_msgs::msg::Cliff>(
      "/roomba/cliff", 10, std::bind(&SystemController::cliffCallback, this, _1));

  this->declare_parameter("status_timeout", 1.0);
  double statusTimeout;
  this->get_parameter("status_timeout", statusTimeout);
  this->_status.SetTimeout(rclcpp::Duration::from_seconds(statusTimeout));
  RCLCPP_INFO(get_logger(), "ON CONFIG");
}

void SystemController::bumperCallback(const create_msgs::msg::Bumper::ConstSharedPtr msg)
{
  auto status = this->_status.GetData();
  status.bumper = *msg;
  this->_status.SetData(status);
}

void SystemController::cliffCallback(const create_msgs::msg::Cliff::ConstSharedPtr msg)
{
    auto status = this->_status.GetData();
    status.cliff = *msg;
    this->_status.SetData(status);
}

bool isWarningZone(const RobotStatus& status)
{
  const auto& bumper = status.bumper;

  return bumper.light_signal_center_left > WarningIntensity || bumper.light_signal_front_left > WarningIntensity ||
         bumper.light_signal_left > WarningIntensity || bumper.light_signal_center_right > WarningIntensity ||
         bumper.light_signal_front_right > WarningIntensity || bumper.light_signal_right > WarningIntensity;
}

bool isStopCondition(const RobotStatus& status)
{
    const auto& bumper = status.bumper;
    const auto& cliff = status.cliff;

    return bumper.is_left_pressed || bumper.is_right_pressed || cliff.is_cliff_left || cliff.is_cliff_right || cliff.is_cliff_front_left || cliff.is_cliff_front_right;
}

void SystemController::cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  auto toSend = std::make_unique<geometry_msgs::msg::Twist>();

  if (this->_status.IsDataTimeout())
  {
    this->_cmdVelPub->publish(std::move(toSend));
    return;
  }

  toSend->angular.z = msg->angular.z;
  const auto status = this->_status.GetData();

  if (isStopCondition(status))
  {
    toSend->linear.x = std::min(0.0, msg->linear.x);
    this->_cmdVelPub->publish(std::move(toSend));
    return;
  }

  if (isWarningZone(status))
  {
    RCLCPP_INFO(this->get_logger(), "WARNING SPEED");
    toSend->linear.x = std::min(WarningSpeed, msg->linear.x);
    this->_cmdVelPub->publish(std::move(toSend));
    return;
  }

  toSend->linear.x = msg->linear.x;
  this->_cmdVelPub->publish(std::move(toSend));
}
