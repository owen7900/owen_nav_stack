#include "owen_navigation/NavigatorNode.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <random>
#include <rclcpp/logger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "owen_navigation/path_followers/PathFollowerFactory.hpp"
#include "owen_navigation/path_generators/PathGeneratorFactory.hpp"
#include "owen_navigation/recovery_behaviours/RecoveryBehaviourFactory.hpp"

namespace Navigation {

namespace Constants {
constexpr double DefaultPoseTimeout = 0.1;
constexpr double DefaultMaxRecoveryTime = 5.0;
}  // namespace Constants

NavigatorNode::NavigatorNode(const std::string& name)
    : rclcpp::Node(name), recoveryBehaviourStartTime(0.0) {
  tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

  this->controlLoopTimer = this->create_wall_timer(
      std::chrono::duration<double>(0.05), [this] { controlLoop(); });
  map = std::make_shared<Mapping::MapManager>(*this);
  if (!map) {
    RCLCPP_FATAL(this->get_logger(), "Map is null");
    throw -1;
  }

  const auto pathGeneratorsStrings = this->get_parameter_or(
      "path_generators", std::vector<std::string>{"AStar"});

  for (const auto& generatorString : pathGeneratorsStrings) {
    auto gen = GetPathGenerator(*this, generatorString, map);
    if (gen) {
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Got path generator: " << generatorString);
      this->pathGenerators.push_back(std::move(gen));
    }
  }

  const auto recoveryBehaviourStrings = this->get_parameter_or(
      "recovery_behaviours", std::vector<std::string>{"back_up"});
  for (const auto& behaviourString : recoveryBehaviourStrings) {
    auto rec = RecoveryBehaviourFactory::GetRecoveryBehaviour(behaviourString,
                                                              *this, map);
    if (rec) {
      this->recoveryBehaviours.push_back(rec);
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Got recovery behaviour: " << behaviourString);
    }
  }

  this->pathFollower = PathFollowers::PathFollowerFactory::GetFollower(
      *this, map,
      this->get_parameter_or<std::string>("path_follower", "pure_pursuit"));

  if (!pathFollower) {
    RCLCPP_FATAL(this->get_logger(), "Path follower is nullptr");
    throw -1;
  }

  commandPub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

  dataTimeout =
      this->get_parameter_or("data_timeout", Constants::DefaultPoseTimeout);
  maxRecoveryTime = this->get_parameter_or("max_recovery_time",
                                           Constants::DefaultMaxRecoveryTime);
}

void NavigatorNode::updatePose() {
  try {
    geometry_msgs::msg::TransformStamped t;
    t = tfBuffer->lookupTransform("map", "base_footprint", tf2::TimePointZero);
    geometry_msgs::msg::Pose in;
    in.orientation.w = 1;
    geometry_msgs::msg::Pose out;
    tf2::doTransform(in, out, t);

    owen_common::types::Pose2D pose;
    pose.x = out.position.x;
    pose.y = out.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(out.orientation, q);

    tf2::Matrix3x3 m(q);
    double pitch, roll;
    m.getEulerYPR(pose.yaw, pitch, roll);
    this->pose.SetData(pose);

  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
                "base_footprint", "map", ex.what());
    return;
  }
}

void NavigatorNode::controlLoop() {
  this->updatePose();
  this->map->UpdateMap(pose.GetDataRef());
  for (const auto& gen : pathGenerators) {
    if (gen->HasNewCommand()) {
      this->activeGenerator = gen;
      this->pathFollower->UpdatePath(gen->GeneratePath(pose.GetDataRef()));
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Got a new command from: " << gen->GetName());
    }
  }

  if (this->activeGenerator && this->activeGenerator->HasUpdatedPath()) {
    this->pathFollower->UpdatePath(
        this->activeGenerator->GeneratePath(pose.GetDataRef()));
    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Got an updated path from: " << this->activeGenerator->GetName());
  }

  if (!this->activeGenerator) {
    RCLCPP_INFO_STREAM(this->get_logger(), "No active path generator");
    this->commandPub->publish(geometry_msgs::msg::Twist{});
  } else if (this->pose.GetDataAge() > dataTimeout) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Have outdated pose, is "
                                               << this->pose.GetDataAge()
                                               << "s old. Stopping vehicle");
    this->commandPub->publish(geometry_msgs::msg::Twist{});
  } else {
    auto command = this->pathFollower->CalculateCommand(pose.GetDataRef());
    if (command.has_value()) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Sending command"
                                                 << command->linear.x << " "
                                                 << command->angular.z);
      this->commandPub->publish(command.value());
      if (this->pathFollower->IsArrived()) {
        activeGenerator = nullptr;
      }
    } else {
      this->executeRecovery();
    }
  }
}

void NavigatorNode::executeRecovery() {
  RCLCPP_INFO(this->get_logger(), "Executing recovery");
  if (recoveryBehaviours.empty()) {
    this->commandPub->publish(geometry_msgs::msg::Twist{});
  }

  const double now = this->get_clock()->now().seconds();

  if (now - this->recoveryBehaviourStartTime > this->maxRecoveryTime ||
      !activeRecoveryBehaviour ||
      !activeRecoveryBehaviour->IsRecoveryCommandActive()) {
    this->recoveryBehaviourStartTime = now;
    activeRecoveryBehaviour = nullptr;

    while (!activeRecoveryBehaviour) {
      const size_t randomRec = std::rand() % recoveryBehaviours.size();

      if (this->recoveryBehaviours[randomRec]->IsRecoveryCommandActive()) {
        activeRecoveryBehaviour = this->recoveryBehaviours[randomRec];
      }
    }
  }

  this->commandPub->publish(activeRecoveryBehaviour->GetRecoveryCommand());
}

}  // namespace Navigation
