
#include "master_navigator/MasterNavigator.hpp"
#include <functional>
#include <roomba_msgs/msg/detail/multifloor_point__struct.hpp>
#include <string>
#include <unordered_map>

namespace Constants
{

std::unordered_map<NavigationState, std::string> MapToString = {
  { NavigationState::WaitingForDestination, "Waiting For Desintation" },
  { NavigationState::ProcessingDestination, "Processing Destination" },
  { NavigationState::NavigateToPoint, "Navigating To Point" },
  { NavigationState::TraverseElevator, "Traverse Elevator" },
};

}

MasterNavigator::MasterNavigator(const std::string& name) : rclcpp::Node(name), log_period_count(0)
{
  this->declare_parameter("/starting_floor", rclcpp::ParameterValue("one"));
  current_pos.floor_id.data = this->get_parameter("/starting_floor").as_string();

  floor_pub = this->create_publisher<std_msgs::msg::String>("/floor", 1);
  elevator_pub = this->create_publisher<std_msgs::msg::String>("/elevator_request", 1);
  feature_list_pub = this->create_publisher<roomba_msgs::msg::StringArray>("/feature_list", 1);
  arrived_pub = this->create_publisher<std_msgs::msg::Bool>("/arrived", 1);

  destination_sub = this->create_subscription<roomba_msgs::msg::MultifloorPoint>(
      "/multifloor_destination", 10, std::bind(&MasterNavigator::destination_callback, this, std::placeholders::_1));

  path_obstacles_srv = this->create_client<roomba_msgs::srv::GetPathObstacles>("/path_obstacles_srv");
  continue_srv = this->create_client<roomba_msgs::srv::CanContinue>("/can_continue_srv");

  navigation_server = rclcpp_action::create_client<NavigateClientT>(
      get_node_base_interface(), get_node_graph_interface(), get_node_logging_interface(),
      get_node_waitables_interface(), "navigate_to_pose");

  elevator_server = rclcpp_action::create_client<ElevatorClientT>(get_node_base_interface(), get_node_graph_interface(),
                                                                  get_node_logging_interface(),
                                                                  get_node_waitables_interface(), "traverse_elevator");

  timer = this->create_wall_timer(std::chrono::duration<double>(0.1), std::bind(&MasterNavigator::control_loop, this));
}

void MasterNavigator::destination_callback(roomba_msgs::msg::MultifloorPoint::ConstSharedPtr msg)
{
  if (state == NavigationState::WaitingForDestination || *msg != destination)
  {
    state = NavigationState::ProcessingDestination;
    destination = *msg;
    this->process_new_destination();
  }
}

void MasterNavigator::control_loop()
{
  // do anything that needs to happen at 10Hz here

  ++log_period_count;
  if (log_period_count == 10)
  {
    log_period_count = 0;
    this->print_status();
  }
}

void MasterNavigator::process_new_destination()
{
  if (!planner)
  {
    planner = std::make_unique<MultifloorPathPlanner>(shared_from_this());
  }
  path = planner->plan_path(destination, current_pos);

  std::cout << "Planned Path: ";
  for (const auto& p : path.points)
  {
    std::cout << "{" << p.floor_id.data << ", " << p.point.x << ", " << p.point.y << "}, ";
  }
  std::cout << std::endl;

  path_idx = 0;

  if (path.points.size() == 0)
  {
    RCLCPP_ERROR(get_logger(), "COULD NOT PLAN PATH TO DESTINATION");
    state = NavigationState::WaitingForDestination;
  }
  else
  {
    state = NavigationState::NavigateToPoint;
    handle_navigation_success();
  }
}

void MasterNavigator::navigate_to_goal(NavigateClientT::Goal& goal)
{
  auto send_goal_options = rclcpp_action::Client<NavigateClientT>::SendGoalOptions();
  send_goal_options.result_callback =
      std::bind(&MasterNavigator::navigation_result_callback, this, std::placeholders::_1);
  send_goal_options.goal_response_callback =
      std::bind(&MasterNavigator::navigation_goal_response_callback, this, std::placeholders::_1);

  future_navigation_handle = navigation_server->async_send_goal(goal, send_goal_options);
}

void MasterNavigator::traverse_elevator(ElevatorClientT::Goal& goal)
{
  auto send_goal_options = rclcpp_action::Client<ElevatorClientT>::SendGoalOptions();
  send_goal_options.result_callback =
      std::bind(&MasterNavigator::elevator_result_callback, this, std::placeholders::_1);
  send_goal_options.goal_response_callback =
      std::bind(&::MasterNavigator::elevator_goal_respose_callback, this, std::placeholders::_1);

  future_elevator_handle = elevator_server->async_send_goal(goal, send_goal_options);
}

void MasterNavigator::print_status() const
{
  RCLCPP_INFO_STREAM(this->get_logger(), "State: " << Constants::MapToString.at(state)
                                                   << " Current Floor: " << current_pos.floor_id.data
                                                   << " Destination: (" << destination.floor_id.data << ", "
                                                   << destination.point.x << ", " << destination.point.y << ")");
}

void MasterNavigator::navigation_result_callback(
    const rclcpp_action::ClientGoalHandle<NavigateClientT>::WrappedResult& result)
{
  if (result.goal_id != future_navigation_handle.get()->get_goal_id())
  {
    RCLCPP_WARN(this->get_logger(), "Got Old Goal Result");
    return;
  }

  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      handle_navigation_success();
      return;
    case rclcpp_action::ResultCode::ABORTED:
    case rclcpp_action::ResultCode::CANCELED:
    case rclcpp_action::ResultCode::UNKNOWN:
    default:
      handle_navigation_failure();
      return;
  }
}

void MasterNavigator::handle_navigation_failure()
{
  RCLCPP_ERROR(get_logger(), "PANICCCCCCC");
  throw -1;
}

void MasterNavigator::handle_navigation_success()
{
  if (state == NavigationState::NavigateToPoint)
  {
    ++path_idx;
    if (path_idx >= path.points.size())
    {
      std_msgs::msg::Bool msg;
      msg.data = true;
      arrived_pub->publish(msg);
      state = NavigationState::WaitingForDestination;
    }
    else if ((path_idx > 0 && path.points[path_idx - 1].floor_id != path.points[path_idx].floor_id) ||
             (path_idx == 0 && path.points[path_idx].floor_id != current_pos.floor_id))
    {
      state = NavigationState::TraverseElevator;
      ElevatorClientT::Goal goal;
      goal.target_floor = path.points[path_idx].floor_id;
      this->traverse_elevator(goal);
    }
    else
    {
      NavigateClientT::Goal goal;
      goal.pose.header.frame_id = "/map";
      goal.pose.header.stamp = this->get_clock()->now();
      goal.pose.pose.position = path.points[path_idx].point;
      this->navigate_to_goal(goal);
    }
  }
  else
  {
    RCLCPP_WARN(get_logger(), "We should not be here, we have finished navigation yet we were not navigating");
  }
}

void MasterNavigator::elevator_result_callback(
    const rclcpp_action::ClientGoalHandle<ElevatorClientT>::WrappedResult& result)

{
  if (result.goal_id != future_elevator_handle.get()->get_goal_id())
  {
    RCLCPP_WARN(this->get_logger(), "Got Old Goal Result");
    return;
  }

  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED: {
      state = NavigationState::NavigateToPoint;
      NavigateClientT::Goal goal;
      goal.pose.header.frame_id = "/map";
      goal.pose.header.stamp = this->get_clock()->now();
      goal.pose.pose.position = path.points[path_idx].point;
      this->navigate_to_goal(goal);
      return;
    }
    case rclcpp_action::ResultCode::ABORTED:
    case rclcpp_action::ResultCode::CANCELED:
    case rclcpp_action::ResultCode::UNKNOWN:
    default:
      RCLCPP_ERROR(get_logger(), "Failed to traverse elevator");
      return;
  }
}

void MasterNavigator::navigation_goal_response_callback(
    const rclcpp_action::ClientGoalHandle<NavigateClientT>::SharedPtr& goal)
{
  if (!goal)
  {
    RCLCPP_ERROR(this->get_logger(), "Navigation failed to start HANDLE THIS FAILURE");
    state = NavigationState::WaitingForDestination;
  }
}

void MasterNavigator::elevator_goal_respose_callback(
    const rclcpp_action::ClientGoalHandle<ElevatorClientT>::SharedPtr& goal)
{
  if (!goal)
  {
    RCLCPP_ERROR(get_logger(), "Elevator traversal failed to start HANDLE THIS FAILURE");
    state = NavigationState::WaitingForDestination;
  }
}
