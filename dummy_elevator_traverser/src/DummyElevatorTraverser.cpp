#include "dummy_elevator_traverser/DummyElevatorTraverser.hpp"

DummyElevatorTraverser::DummyElevatorTraverser(const std::string& name) : rclcpp::Node(name)
{
  this->_actionServer = rclcpp_action::create_server<TraverseElevator>(
      this, "traverse_elevator",
      std::bind(&DummyElevatorTraverser::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DummyElevatorTraverser::handleCancel, this, std::placeholders::_1),
      std::bind(&DummyElevatorTraverser::handleAccepted, this, std::placeholders::_1));

  this->_cmdVelPub = this->create_publisher<geometry_msgs::msg::Twist>("/elevator_traverser/cmd_vel", 1);
  this->_tagSub = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
      "/detections", 10, std::bind(&DummyElevatorTraverser::tagDetectionCallback, this, std::placeholders::_1));

  this->_timer =
      this->create_wall_timer(std::chrono::duration<double>(0.05), std::bind(&DummyElevatorTraverser::execute, this));
}

rclcpp_action::GoalResponse DummyElevatorTraverser::handleGoal(const rclcpp_action::GoalUUID& uuid,
                                                               std::shared_ptr<const TraverseElevator::Goal> goal)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Got goal request " << goal->target_floor.data);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
DummyElevatorTraverser::handleCancel(const std::shared_ptr<GoalHandleTraverseElevator> goal_handle)
{
  if (goal_handle == this->_goalHandle)
  {
    this->reset();
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  RCLCPP_ERROR_STREAM(this->get_logger(), "Cancelling non-active goal " << goal_handle->get_goal()->target_floor.data);
  return rclcpp_action::CancelResponse::REJECT;
}

void DummyElevatorTraverser::handleAccepted(const std::shared_ptr<GoalHandleTraverseElevator> goal_handle)
{
  this->reset();
  this->_goalHandle = goal_handle;
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Exectuing goal with target floor " << this->_goalHandle->get_goal()->target_floor.data);
  this->_timer->reset();
}

void DummyElevatorTraverser::execute()
{
  const auto tagElapsed = this->get_clock()->now() - this->_lastTagTime;
  if (this->_hasSeenTag && tagElapsed.seconds() > 5)
  {
    TraverseElevator::Result::SharedPtr result;
    this->_goalHandle->succeed(result);
    this->_timer->cancel();
  }
}

void DummyElevatorTraverser::reset()
{
  if (!this->_timer->is_canceled())
  {
    this->_timer->cancel();
  }
  this->_goalHandle = nullptr;
  this->_hasSeenTag = false;
  this->_lastTagTime = this->get_clock()->now();
}

void DummyElevatorTraverser::tagDetectionCallback(
    const apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr detections)
{
  if (!detections->detections.empty())
  {
    this->_hasSeenTag = true;
    this->_lastTagTime = this->get_clock()->now();
  }
}
