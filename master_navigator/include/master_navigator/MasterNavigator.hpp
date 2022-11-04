#pragma once

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_util/simple_action_server.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "roomba_msgs/msg/multifloor_point.hpp"
#include "roomba_msgs/msg/string_array.hpp"
#include "roomba_msgs/srv/can_continue.hpp"
#include "roomba_msgs/srv/get_elevator_position.hpp"
#include "roomba_msgs/srv/get_path_obstacles.hpp"

enum class NavigationState
{
  WaitingForDestination,
  ProcessingDestination,
  NavigateToElevator,
  NavigateToGoal,
  GetIntoElevator,
  ExitElevator,
  WaitingForElevatorLocation,
};

class MasterNavigator : public rclcpp::Node
{
public:
  using ClientT = nav2_msgs::action::NavigateToPose;

public:
  MasterNavigator(const std::string& name);

private:
  void destination_callback(roomba_msgs::msg::MultifloorPoint::ConstSharedPtr msg);
  void navigation_result_callback(const rclcpp_action::ClientGoalHandle<ClientT>::WrappedResult& result);
  void navigation_goal_response_callback(const rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr& goal);
  void elevator_pos_callback(rclcpp::Client<roomba_msgs::srv::GetElevatorPosition>::SharedFuture resp);

  void handle_navigation_success();
  void handle_navigation_failure();

  void control_loop();
  void process_new_destination();
  void start_elevator_navigation();
  void start_goal_navigation();
  void print_status() const;

  void navigate_to_goal(ClientT::Goal& goal);

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr floor_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_pub;
  rclcpp::Publisher<roomba_msgs::msg::StringArray>::SharedPtr feature_list_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arrived_pub;

  rclcpp::Subscription<roomba_msgs::msg::MultifloorPoint>::SharedPtr destination_sub;

  rclcpp::Client<roomba_msgs::srv::GetPathObstacles>::SharedPtr path_obstacles_srv;
  rclcpp::Client<roomba_msgs::srv::CanContinue>::SharedPtr continue_srv;
  rclcpp::Client<roomba_msgs::srv::GetElevatorPosition>::SharedPtr elevator_pos_srv;

  rclcpp_action::Client<ClientT>::SharedPtr navigation_server;
  std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr> future_goal_handle;

  NavigationState state;
  roomba_msgs::msg::MultifloorPoint destination;
  uint8_t log_period_count;
  std::string current_floor;
};
