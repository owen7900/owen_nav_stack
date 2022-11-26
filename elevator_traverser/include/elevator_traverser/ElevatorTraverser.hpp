#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "elevator_traverser/Configuration.hpp"
#include "roomba_msgs/action/traverse_elevator.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

enum class TraversalState
{
  Initialization,
  AlignWithOutside,
  WaitOutside,
  AlignWithInside,
  RotateToFaceDoor,
  WaitForDoorToClose,
  WaitForDoorToOpen,
  LineUpWithExit,
  ExitElevator
};

struct TraversalStatus
{
  int32_t door_id;
};

class ElevatorTraverser : public rclcpp::Node
{
public:
  using TraverseElevator = roomba_msgs::action::TraverseElevator;
  using GoalHandleTraverseElevator = rclcpp_action::ServerGoalHandle<TraverseElevator>;
  ElevatorTraverser(const std::string& name);

private:
  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid,
                                         std::shared_ptr<const TraverseElevator::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleTraverseElevator> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandleTraverseElevator> goal_handle);

  void tagDetectionCallback(const apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr msgs);

  void execute();
  void reset();

  bool initializeTraversal();
  bool lineUpWithExit();
  bool exitElevator();

  std::pair<double, double> getTagDistanceAndAngleOffset(const int32_t tagId) const;

  // returns true if aligned with tag
  bool alignToTag(int32_t id);
  bool canSeeTag(int32_t id);
  bool rotateToFaceDoor(int32_t id);

private:
  rclcpp_action::Server<TraverseElevator>::SharedPtr _actionServer;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmdVelPub;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr _tagSub;
  std::shared_ptr<GoalHandleTraverseElevator> _goalHandle;
  apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr _detections;

  rclcpp::TimerBase::SharedPtr _timer;

  TraversalState _state;
  Configuration _config;
  TraversalStatus _status;

  double _kAngle;
  double _kDistance;
  double _goalTolerance;
  double _exitTraversalDuration;
  std::shared_ptr<tf2_ros::TransformListener> _transformListener;
  std::unique_ptr<tf2_ros::Buffer> _tfBuffer;

  double _exitStartTime;
};
