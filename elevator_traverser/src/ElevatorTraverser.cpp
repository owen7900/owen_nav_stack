#include "elevator_traverser/ElevatorTraverser.hpp"

#include <functional>

ElevatorTraverser::ElevatorTraverser(const std::string& name) : rclcpp::Node(name)
{
  this->declare_parameter("elevator_config_file", rclcpp::ParameterType::PARAMETER_STRING);
  std::string elevator_config_file = "/home/owen/owen_ws/src/owen_nav_stack/owen_bringup/config/elevator.yaml";
  this->get_parameter("elevator_config_file", elevator_config_file);
  _config.load(elevator_config_file);

  this->declare_parameter("k_angle", 0.2);
  this->get_parameter("k_angle", this->_kAngle);
  this->declare_parameter("k_distance", 0.2);
  this->get_parameter("k_distance", this->_kDistance);
  this->declare_parameter("goal_tolerance", 0.1);
  this->get_parameter("goal_tolerance", this->_goalTolerance);

  this->_actionServer = rclcpp_action::create_server<TraverseElevator>(
      this, "traverse_elevator",
      std::bind(&ElevatorTraverser::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ElevatorTraverser::handleCancel, this, std::placeholders::_1),
      std::bind(&ElevatorTraverser::handleAccepted, this, std::placeholders::_1));

  this->_cmdVelPub = this->create_publisher<geometry_msgs::msg::Twist>("/elevator_traverser/cmd_vel", 1);
  this->_tagSub = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
      "/detections", 10, std::bind(&ElevatorTraverser::tagDetectionCallback, this, std::placeholders::_1));

  _timer = this->create_wall_timer(std::chrono::duration<double>(0.05), std::bind(&ElevatorTraverser::execute, this));
  // timer->cancel(); // uncomment this when testing is done
  // reset();
  this->_state = TraversalState::Initialization;
}

void ElevatorTraverser::reset()
{
  if (!_timer->is_canceled())
  {
    // this->timer->cancel();
  }
  // this->goal_handle_ = nullptr;
  // this->state = TraversalState::Initialization;
}

rclcpp_action::GoalResponse ElevatorTraverser::handleGoal(const rclcpp_action::GoalUUID& uuid,
                                                          std::shared_ptr<const TraverseElevator::Goal> goal)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Got goal request " << goal->target_floor.data);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
ElevatorTraverser::handleCancel(const std::shared_ptr<GoalHandleTraverseElevator> goal_handle)
{
  if (goal_handle == this->_goalHandle)
  {
    this->reset();
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  RCLCPP_ERROR_STREAM(this->get_logger(), "Cancelling non-active goal " << goal_handle->get_goal()->target_floor.data);
  return rclcpp_action::CancelResponse::REJECT;
}

void ElevatorTraverser::handleAccepted(const std::shared_ptr<GoalHandleTraverseElevator> goal_handle)
{
  this->reset();
  _goalHandle = goal_handle;
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Exectuing goal with target floor " << _goalHandle->get_goal()->target_floor.data);
  this->_timer->reset();
}

void ElevatorTraverser::execute()
{
  if (!this->_goalHandle)
  {
    // return;  UNCOMMENT THIS WHEN INTEGRATING
  }
  switch (_state)
  {
    case TraversalState::Initialization:
      if (this->initializeTraversal())
      {
        std::cout << "Transition to allign with outside" << std::endl;
        _state = TraversalState::AlignWithOutside;
      }
      break;
    case TraversalState::AlignWithOutside:
      if (this->alignToTag(_status.door_id))
      {
        std::cout << "Transition to wait outside" << std::endl;
        _state = TraversalState::WaitOutside;
      }
      break;
    case TraversalState::WaitOutside:
      if (this->canSeeTag(_config.GetElevatorFromDoorTag(_status.door_id).backId))
      {
        std::cout << "Transition to inside alligh" << std::endl;
        _state = TraversalState::AlignWithInside;
      }
      break;
    case TraversalState::AlignWithInside:
      if (this->alignToTag(_config.GetElevatorFromDoorTag(_status.door_id).backId))
      {
        std::cout << "Transition to rotate to face door" << std::endl;
        _state = TraversalState::RotateToFaceDoor;
      }
      break;

    case TraversalState::RotateToFaceDoor:
      if (this->rotateToFaceDoor())
      {
        std::cout << "Transition to wait for door close" << std::endl;
        _state = TraversalState::WaitForDoorToClose;
      }
      break;
    case TraversalState::WaitForDoorToClose:
      if (this->canSeeTag(_config.GetElevatorFromDoorTag(_status.door_id).doorId))
      {
        std::cout << "Transition to WaitForDoorToOpen" << std::endl;
        _state = TraversalState::WaitForDoorToOpen;
      }
      break;
    case TraversalState::WaitForDoorToOpen:
      if (!this->canSeeTag(_config.GetElevatorFromDoorTag(_status.door_id).doorId))
      {
        std::cout << "Transition to exit elevator" << std::endl;
        _state = TraversalState::ExitElevator;
      }
      break;
    case TraversalState::ExitElevator:
      if (this->exitElevator())
      {
        // TraverseElevator::Result::SharedPtr result;
        // this->goal_handle_->succeed(result);
        // this->timer->cancel();
      }
      break;
  }
}

bool ElevatorTraverser::initializeTraversal()
{
  if (!this->_detections)
  {
    RCLCPP_ERROR(this->get_logger(), "HAVE NOT RECIEVED ANY TAG DETECTIONS");
    return false;
  }
  int32_t door_id = -1;
  bool can_see_door = false;

  for (const auto& det : this->_detections->detections)
  {
    if (_config.IsElevatorDoorTag(det.id))
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

  _status.door_id = door_id;
  return true;
}

bool ElevatorTraverser::exitElevator()
{
  return true;
}

bool ElevatorTraverser::alignToTag(int32_t id)
{
  geometry_msgs::msg::Twist cmd;
  if (!this->canSeeTag(id))
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot see tag %d for traveral", id);
    this->_cmdVelPub->publish(cmd);
    return false;
  }
  // TODO get these from the detections
  double diffAngle = 0.0, diffPos = 0.0;

  cmd.linear.x = diffPos * this->_kDistance;
  cmd.angular.z = diffAngle * this->_kAngle * (cmd.linear.x >= 0 ? 1.0 : -1.0);
  this->_cmdVelPub->publish(cmd);

  return diffPos < this->_goalTolerance;
}

bool ElevatorTraverser::rotateToFaceDoor()
{
  return true;
}

bool ElevatorTraverser::canSeeTag(int32_t id)
{
  if (!this->_detections)
  {
    return false;
  }
  for (const auto& d : this->_detections->detections)
  {
    if (d.id == id)
    {
      return true;
    }
  }

  return false;
}

void ElevatorTraverser::tagDetectionCallback(
    const apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr detections)
{
  this->_detections = detections;
}
