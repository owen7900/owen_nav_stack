#include "owen_common/timestamp.hpp"
#include "rclcpp/rclcpp.hpp"
#include "create_msgs/msg/bumper.hpp"
#include "create_msgs/msg/cliff.hpp"
#include "geometry_msgs/msg/twist.hpp"

struct RobotStatus
{
  create_msgs::msg::Bumper bumper;
  create_msgs::msg::Cliff cliff;
};

class SystemController : public rclcpp::Node
{
public:
  SystemController(const std::string& name);

private:
  void bumperCallback(const create_msgs::msg::Bumper::ConstSharedPtr msg);
  void cliffCallback(const create_msgs::msg::Cliff::ConstSharedPtr msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg);

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmdVelPub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmdVelSub;
  rclcpp::Subscription<create_msgs::msg::Bumper>::SharedPtr _bumperSub;
  rclcpp::Subscription<create_msgs::msg::Cliff>::SharedPtr _cliffSub;
  owen_common::Timestamp<RobotStatus> _status;
};
