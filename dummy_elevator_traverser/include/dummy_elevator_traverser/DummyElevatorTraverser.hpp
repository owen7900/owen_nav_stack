#ifndef DUMMY_ELEVATOR_TRAVERSER_HPP_
#define DUMMY_ELEVATOR_TRAVERSER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "roomba_msgs/action/traverse_elevator.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class DummyElevatorTraverser : public rclcpp::Node
{
public:
  using TraverseElevator = roomba_msgs::action::TraverseElevator;
  using GoalHandleTraverseElevator = rclcpp_action::ServerGoalHandle<TraverseElevator>;
  DummyElevatorTraverser(const std::string& name);

private:
  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid,
                                         std::shared_ptr<const TraverseElevator::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleTraverseElevator> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandleTraverseElevator> goal_handle);

  void tagDetectionCallback(const apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr msgs);

  void execute();
  void reset();

private:
  rclcpp_action::Server<TraverseElevator>::SharedPtr _actionServer;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmdVelPub;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr _tagSub;
  std::shared_ptr<GoalHandleTraverseElevator> _goalHandle;
  rclcpp::TimerBase::SharedPtr _timer;

  rclcpp::Time _lastTagTime;
  bool _hasSeenTag;
};

#endif
