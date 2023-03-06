#pragma once

#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_util/simple_action_server.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "roomba_msgs/msg/multifloor_point.hpp"
#include "roomba_msgs/msg/string_array.hpp"
#include "roomba_msgs/srv/can_continue.hpp"
#include "roomba_msgs/srv/get_elevator_position.hpp"
#include "roomba_msgs/srv/get_path_obstacles.hpp"
#include "roomba_msgs/action/traverse_elevator.hpp"

#include "master_navigator/MultifloorPathPlanner.hpp"

enum class NavigationState
{
  WaitingForDestination,
  ProcessingDestination,
  NavigateToPoint,
  TraverseElevator,
};

class MasterNavigator : public rclcpp::Node
{
public:
  using NavigateClientT = nav2_msgs::action::NavigateToPose;
  using ElevatorClientT = roomba_msgs::action::TraverseElevator;

public:
  MasterNavigator(const std::string& name);

private:
  void destination_callback(roomba_msgs::msg::MultifloorPoint::ConstSharedPtr msg);

  void navigation_result_callback(const rclcpp_action::ClientGoalHandle<NavigateClientT>::WrappedResult& result);
  void navigation_feedback_callback(const rclcpp_action::ClientGoalHandle<NavigateClientT>::SharedPtr,
                                    const std::shared_ptr<const NavigateClientT::Feedback> feedback);
  void navigation_goal_response_callback(const rclcpp_action::ClientGoalHandle<NavigateClientT>::SharedPtr& goal);

  void elevator_result_callback(const rclcpp_action::ClientGoalHandle<ElevatorClientT>::WrappedResult& result);
  void elevator_goal_respose_callback(const rclcpp_action::ClientGoalHandle<ElevatorClientT>::SharedPtr& goal);

  void robot_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped& pose);

  void handle_navigation_success();
  void handle_navigation_failure();

  void send_spoken_instruction() const;

  void control_loop();
  void process_new_destination();
  void print_status() const;

  void navigate_to_goal(NavigateClientT::Goal& goal);
  void traverse_elevator(ElevatorClientT::Goal& goal);

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr floor_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_pub;
  rclcpp::Publisher<roomba_msgs::msg::StringArray>::SharedPtr feature_list_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arrived_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr speak_pub;

  rclcpp::Subscription<roomba_msgs::msg::MultifloorPoint>::SharedPtr destination_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub;

  rclcpp::Client<roomba_msgs::srv::GetPathObstacles>::SharedPtr path_obstacles_srv;
  rclcpp::Client<roomba_msgs::srv::CanContinue>::SharedPtr continue_srv;

  rclcpp_action::Client<NavigateClientT>::SharedPtr navigation_server;
  std::shared_future<rclcpp_action::ClientGoalHandle<NavigateClientT>::SharedPtr> future_navigation_handle;

  rclcpp_action::Client<ElevatorClientT>::SharedPtr elevator_server;
  std::shared_future<rclcpp_action::ClientGoalHandle<ElevatorClientT>::SharedPtr> future_elevator_handle;

  rclcpp::TimerBase::SharedPtr timer;

  NavigationState state;
  roomba_msgs::msg::MultifloorPoint destination;
  roomba_msgs::msg::MultifloorPoint current_pos;
  roomba_msgs::msg::MultifloorPath path;
  uint path_idx;
  uint8_t log_period_count;
  std::unique_ptr<MultifloorPathPlanner> planner;
};
