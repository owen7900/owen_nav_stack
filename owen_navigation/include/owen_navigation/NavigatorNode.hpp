#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "owen_common/TimestampData.hpp"
#include "owen_navigation/mapping/MapManager.hpp"
#include "owen_navigation/path_followers/BasePathFollower.hpp"
#include "owen_navigation/path_generators/BasePathGenerator.hpp"

namespace Navigation {

class NavigatorNode : public rclcpp::Node {
 public:
  using Pose = owen_common::types::BasePose2D<double>;

 public:
  explicit NavigatorNode(const std::string& name);

 private:
  void controlLoop();

  void executeRecovery();

 private:
  std::vector<std::shared_ptr<PathGenerators::BasePathGenerator>>
      pathGenerators;

  std::shared_ptr<PathGenerators::BasePathGenerator> activeGenerator;

  std::shared_ptr<Mapping::MapManager> map;

  rclcpp::TimerBase::SharedPtr controlLoopTimer;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr commandPub;

  std::unique_ptr<PathFollowers::BasePathFollower> pathFollower;

  owen_common::TimeoutData<Pose> pose;

  double dataTimeout;
};

}  // namespace Navigation
