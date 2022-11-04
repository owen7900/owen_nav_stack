
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
  { NavigationState::NavigateToElevator, "Navigating To Elevator" },
  { NavigationState::NavigateToGoal, "Navigating To Goal" },
  { NavigationState::GetIntoElevator, "Getting Into Elevator" },
  { NavigationState::ExitElevator, "Exiting Elevator" },
  { NavigationState::WaitingForElevatorLocation, "Waiting for Elevator Location" }
};

}

MasterNavigator::MasterNavigator(const std::string& name) : rclcpp::Node(name), log_period_count(0)
{
  current_floor = this->get_parameter("/starting_floor").as_string();

  floor_pub = this->create_publisher<std_msgs::msg::String>("/floor", 1);
  elevator_pub = this->create_publisher<std_msgs::msg::String>("/elevator_request", 1);
  feature_list_pub = this->create_publisher<roomba_msgs::msg::StringArray>("/feature_list", 1);
  arrived_pub = this->create_publisher<std_msgs::msg::Bool>("/arrived", 1);

  destination_sub = this->create_subscription<roomba_msgs::msg::MultifloorPoint>(
      "/multifloor_destination", 10, std::bind(&MasterNavigator::destination_callback, this, std::placeholders::_1));

  path_obstacles_srv = this->create_client<roomba_msgs::srv::GetPathObstacles>("/path_obstacles_srv");
  continue_srv = this->create_client<roomba_msgs::srv::CanContinue>("/can_continue_srv");

  navigation_server = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      get_node_base_interface(), get_node_graph_interface(), get_node_logging_interface(),
      get_node_waitables_interface(), "navigate_to_pose");

  const auto timer =
      this->create_wall_timer(std::chrono::duration<double>(0.1), std::bind(&MasterNavigator::control_loop, this));
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
  if (destination.floor_id.data != this->current_floor)
  {
    this->start_elevator_navigation();
  }
  else
  {
    state = NavigationState::NavigateToGoal;
    this->start_goal_navigation();
  }
}

void MasterNavigator::start_elevator_navigation()
{
  state = NavigationState::WaitingForElevatorLocation;
  roomba_msgs::srv::GetElevatorPosition::Request::SharedPtr req =
      std::make_shared<roomba_msgs::srv::GetElevatorPosition::Request>();
  req->floor_id.data = this->current_floor;
  elevator_pos_srv->async_send_request(req,
                                       std::bind(&MasterNavigator::elevator_pos_callback, this, std::placeholders::_1));
}

void MasterNavigator::start_goal_navigation()
{
  ClientT::Goal goal;
  goal.pose.pose.position = destination.point;
  goal.pose.header.frame_id = "map";
  this->navigate_to_goal(goal);
}

void MasterNavigator::navigate_to_goal(ClientT::Goal& goal)
{
  auto send_goal_options = rclcpp_action::Client<ClientT>::SendGoalOptions();
  send_goal_options.result_callback =
      std::bind(&MasterNavigator::navigation_result_callback, this, std::placeholders::_1);
  send_goal_options.goal_response_callback =
      std::bind(&MasterNavigator::navigation_goal_response_callback, this, std::placeholders::_1);

  future_goal_handle = navigation_server->async_send_goal(goal, send_goal_options);
}
void MasterNavigator::print_status() const
{
  RCLCPP_INFO_STREAM(this->get_logger(), "State: " << Constants::MapToString.at(state)
                                                   << " Current Floor: " << current_floor << " Destination: ("
                                                   << destination.floor_id.data << ", " << destination.point.x << ", "
                                                   << destination.point.y << ")");
}

void MasterNavigator::navigation_result_callback(const rclcpp_action::ClientGoalHandle<ClientT>::WrappedResult& result)
{
  if (result.goal_id != future_goal_handle.get()->get_goal_id())
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
      handle_navigation_failure();
      return;
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
  if (state == NavigationState::NavigateToGoal)
  {
    std_msgs::msg::Bool msg;
    msg.data = true;
    arrived_pub->publish(msg);
    state = NavigationState::WaitingForDestination;
  }
  else if (state == NavigationState::NavigateToElevator)
  {
    // We are outside elevator
  }
  else
  {
    RCLCPP_WARN(get_logger(), "We should not be here, we have finished navigation yet we were not navigating");
  }
}

void MasterNavigator::navigation_goal_response_callback(const rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr& goal)
{
  if (!goal)
  {
    RCLCPP_ERROR(this->get_logger(), "Navigation failed to start HANDLE THIS FAILURE");
    state = NavigationState::WaitingForDestination;
  }
}

void MasterNavigator::elevator_pos_callback(rclcpp::Client<roomba_msgs::srv::GetElevatorPosition>::SharedFuture resp)
{
  if (state == NavigationState::WaitingForElevatorLocation && resp.valid())
  {
    state = NavigationState::NavigateToElevator;
    auto resp_ptr = resp.get();
    ClientT::Goal goal;
    goal.pose.pose.position = resp_ptr->point;
    goal.pose.header.frame_id = "map";
    this->navigate_to_goal(goal);
  }
  else
  {
    RCLCPP_WARN(get_logger(), "IN ILLEGAL LOCATION, Got elevator pos when not waiting for it");
  }
}
