#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "elevator_traverser/Configuration.hpp"
#include "roomba_msgs/action/traverse_elevator.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"

enum class TraversalState
{
  Initialization,
  AlignWithOutside,
  WaitOutside,
  AlignWithInside,
  RotateToFaceDoor,
  WaitForDoorToClose,
  WaitForDoorToOpen,
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
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const TraverseElevator::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleTraverseElevator> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleTraverseElevator> goal_handle);

  void tag_detection_callback(const apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr msgs);

  void execute();
  void reset();

  bool rotate_to_face_door();
  bool initialize_traversal();
  bool exit_elevator();

  // returns true if aligned with tag
  bool align_to_tag(int32_t id);
  bool can_see_tag(int32_t id);

private:
  rclcpp_action::Server<TraverseElevator>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr tag_sub_;
  std::shared_ptr<GoalHandleTraverseElevator> goal_handle_;
  apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr detections_;

  rclcpp::TimerBase::SharedPtr timer;

  TraversalState state;
  Configuration config;
  TraversalStatus status;
};
