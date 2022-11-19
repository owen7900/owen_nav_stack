#include "elevator_traverser/ElevatorTraverser.hpp"

#include <functional>

ElevatorTraverser::ElevatorTraverser(const std::string& name) : rclcpp::Node(name)
{
  this->declare_parameter("elevator_config_file", rclcpp::ParameterType::PARAMETER_STRING);
  std::string elevator_config_file = "/home/owen/owen_ws/src/owen_nav_stack/owen_bringup/config/elevator.yaml";
  this->get_parameter("elevator_config_file", elevator_config_file);

  config.load(elevator_config_file);
  this->action_server_ = rclcpp_action::create_server<TraverseElevator>(
      this, "traverse_elevator",
      std::bind(&ElevatorTraverser::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ElevatorTraverser::handle_cancel, this, std::placeholders::_1),
      std::bind(&ElevatorTraverser::handle_accepted, this, std::placeholders::_1));

  this->cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/elevator_traverser/cmd_vel", 1);
  this->tag_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
      "/apriltag/detections", 10, std::bind(&ElevatorTraverser::tag_detection_callback, this, std::placeholders::_1));

  timer = this->create_wall_timer(std::chrono::duration<double>(0.05), std::bind(&ElevatorTraverser::execute, this));
  timer->cancel();

  reset();
}

void ElevatorTraverser::reset()
{
  if (!timer->is_canceled())
  {
    this->timer->cancel();
  }
  this->goal_handle_ = nullptr;
  this->state = TraversalState::Initialization;
}

rclcpp_action::GoalResponse ElevatorTraverser::handle_goal(const rclcpp_action::GoalUUID& uuid,
                                                           std::shared_ptr<const TraverseElevator::Goal> goal)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Got goal request " << goal->target_floor.data);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse
ElevatorTraverser::handle_cancel(const std::shared_ptr<GoalHandleTraverseElevator> goal_handle)
{
  if (goal_handle == this->goal_handle_)
  {
    this->reset();
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  RCLCPP_ERROR_STREAM(this->get_logger(), "Cancelling non-active goal " << goal_handle->get_goal()->target_floor.data);
  return rclcpp_action::CancelResponse::REJECT;
}
void ElevatorTraverser::handle_accepted(const std::shared_ptr<GoalHandleTraverseElevator> goal_handle)
{
  this->reset();
  goal_handle_ = goal_handle;
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Exectuing goal with target floor " << goal_handle_->get_goal()->target_floor.data);
  this->timer->reset();
}

void ElevatorTraverser::execute()
{
  if (!this->goal_handle_)
  {
    return;
  }
  switch (state)
  {
    case TraversalState::Initialization:
      if (this->initialize_traversal())
      {
        state = TraversalState::AlignWithOutside;
      }
      break;
    case TraversalState::AlignWithOutside:
      if (this->align_to_tag(status.door_id))
      {
        state = TraversalState::WaitOutside;
      }
      break;
    case TraversalState::WaitOutside:
      if (this->can_see_tag(config.get_elevator_from_door_tag(status.door_id).back_id))
      {
        state = TraversalState::AlignWithInside;
      }
      break;
    case TraversalState::AlignWithInside:
      if (this->align_to_tag(config.get_elevator_from_door_tag(status.door_id).back_id))
      {
        state = TraversalState::RotateToFaceDoor;
      }
      break;

    case TraversalState::RotateToFaceDoor:
      if (this->rotate_to_face_door())
      {
        state = TraversalState::WaitForDoorToClose;
      }
      break;
    case TraversalState::WaitForDoorToClose:
      if (this->can_see_tag(config.get_elevator_from_door_tag(status.door_id).door_id))
      {
        state = TraversalState::WaitForDoorToOpen;
      }
      break;
    case TraversalState::WaitForDoorToOpen:
      if (!this->can_see_tag(config.get_elevator_from_door_tag(status.door_id).door_id))
      {
        state = TraversalState::ExitElevator;
      }
      break;
    case TraversalState::ExitElevator:
      if (this->exit_elevator())
      {
        TraverseElevator::Result::SharedPtr result;
        this->goal_handle_->succeed(result);
        this->timer->cancel();
      }
      break;
  }
}

bool ElevatorTraverser::initialize_traversal()
{
  if (!this->detections_)
  {
    RCLCPP_ERROR(this->get_logger(), "HAVE NOT RECIEVED ANY TAG DETECTIONS");
    return false;
  }
  int32_t door_id = -1;
  bool can_see_door = false;

  for (const auto& det : this->detections_->detections)
  {
    if (config.is_elevator_door_tag(det.id))
    {
      door_id = det.id;
      can_see_door = true;
      break;
    }
  }

  if (!can_see_door)
  {
    RCLCPP_ERROR(this->get_logger(), "Can not see the door, will keep waiting");
    return false;
  }

  status.door_id = door_id;
  return true;
}

bool ElevatorTraverser::exit_elevator()
{
  return true;
}

bool ElevatorTraverser::align_to_tag(int32_t id)
{
  return true;
}

bool ElevatorTraverser::rotate_to_face_door()
{
  return true;
}

bool ElevatorTraverser::can_see_tag(int32_t id)
{
  if (!this->detections_)
  {
    return false;
  }
  for (const auto& d : this->detections_->detections)
  {
    if (d.id == id)
    {
      return true;
    }
  }

  return false;
}

void ElevatorTraverser::tag_detection_callback(
    const apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr detections)
{
  this->detections_ = detections;
}
