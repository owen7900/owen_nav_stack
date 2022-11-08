#include "system_controller/SystemController.hpp"
#include <algorithm>
#include <chrono>
#include <ratio>

constexpr double WarningSpeed = 0.05;
constexpr uint16_t WarningIntensity = 100;

using std::placeholders::_1;

SystemController::SystemController(const std::string& name) : rclcpp::Node(name)
{
  this->_cmdVelPub = this->create_publisher<geometry_msgs::msg::Twist>("/roomba/cmd_vel", 10);

  this->_cmdVelSub = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&SystemController::cmdVelCallback, this, _1));

  this->_manualSub = this->create_subscription<geometry_msgs::msg::Twist>(
      "/manual/cmd_vel", 10, std::bind(&SystemController::manualCmdVelCallback, this, _1));

  this->_bumperSub = this->create_subscription<create_msgs::msg::Bumper>(
      "/roomba/bumper", 10, std::bind(&SystemController::bumperCallback, this, _1));

  this->_cliffSub = this->create_subscription<create_msgs::msg::Cliff>(
      "/roomba/cliff", 10, std::bind(&SystemController::cliffCallback, this, _1));

  this->declare_parameter("status_timeout", 1.0);
  double statusTimeout;
  this->get_parameter("status_timeout", statusTimeout);
  this->_status.SetTimeout(rclcpp::Duration::from_seconds(statusTimeout));

  this->declare_parameter("manual_timeout", 1.0);
  double manualTimeout;
  this->get_parameter("manual_timeout", manualTimeout);
  this->_manualCmd.SetTimeout(rclcpp::Duration::from_seconds(manualTimeout));

  this->declare_parameter("autonomous_timeout", 1.0);
  double autonomousTimeout;
  this->get_parameter("autonomous_timeout", autonomousTimeout);
  this->_autonomousCmd.SetTimeout(rclcpp::Duration::from_seconds(autonomousTimeout));

  this->declare_parameter("control_period", 0.01);
  double controlPeriod;
  this->get_parameter("control_period", controlPeriod);
  this->_timer = this->create_wall_timer(std::chrono::duration<double>(controlPeriod),
                                         std::bind(&SystemController::controlCallback, this));
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

  return bumper.is_left_pressed || bumper.is_right_pressed || cliff.is_cliff_left || cliff.is_cliff_right ||
         cliff.is_cliff_front_left || cliff.is_cliff_front_right;
}

void SystemController::manualCmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  this->_manualCmd.SetData(*msg);
}

void SystemController::cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  this->_autonomousCmd.SetData(*msg);
}

void SystemController::controlCallback()
{
  if (this->_manualCmd.IsDataTimeout())
  {
    auto toSend = std::make_unique<geometry_msgs::msg::Twist>();

    if (this->_autonomousCmd.IsDataTimeout())
    {
      this->_cmdVelPub->publish(std::move(toSend));
      return;
    }

    const auto msg = this->_autonomousCmd.GetData();

    toSend->angular.z = msg.angular.z;
    const auto status = this->_status.GetData();
    if (!this->_status.IsDataTimeout())
    {
      if (isStopCondition(status))
      {
        toSend->linear.x = std::min(0.0, msg.linear.x);
        this->_cmdVelPub->publish(std::move(toSend));
        return;
      }

      if (isWarningZone(status))
      {
        RCLCPP_INFO(this->get_logger(), "WARNING SPEED");
        toSend->linear.x = std::min(WarningSpeed, msg.linear.x);
        this->_cmdVelPub->publish(std::move(toSend));
        return;
      }
    }

    toSend->linear.x = msg.linear.x;
    this->_cmdVelPub->publish(std::move(toSend));
  }
  else
  {
    this->_cmdVelPub->publish(this->_manualCmd.GetData());
  }
}
