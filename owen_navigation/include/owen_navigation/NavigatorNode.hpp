#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "owen_common/TimestampData.hpp"
#include "owen_navigation/mapping/MapManager.hpp"
#include "owen_navigation/path_followers/BasePathFollower.hpp"
#include "owen_navigation/path_generators/BasePathGenerator.hpp"
#include "owen_navigation/recovery_behaviours/BaseRecoveryBehaviour.hpp"

namespace Navigation {

class NavigatorNode : public rclcpp::Node {
 public:
  using Pose = owen_common::types::BasePose2D<double>;

 public:
  explicit NavigatorNode(const std::string& name);

 private:
  void controlLoop();

  void executeRecovery();

  void updatePose();

 private:
  std::vector<std::shared_ptr<PathGenerators::BasePathGenerator>>
      pathGenerators;

  std::shared_ptr<PathGenerators::BasePathGenerator> activeGenerator;

  std::vector<std::shared_ptr<RecoveryBehaviours::BaseRecoveryBehaviour>>
      recoveryBehaviours;

  std::shared_ptr<RecoveryBehaviours::BaseRecoveryBehaviour>
      activeRecoveryBehaviour;

  double recoveryBehaviourStartTime;

  std::shared_ptr<Mapping::MapManager> map;

  rclcpp::TimerBase::SharedPtr controlLoopTimer;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr commandPub;

  std::unique_ptr<PathFollowers::BasePathFollower> pathFollower;

  std::shared_ptr<tf2_ros::TransformListener> tfListener;
  std::unique_ptr<tf2_ros::Buffer> tfBuffer;

  owen_common::TimeoutData<Pose> pose;

  double dataTimeout;
  double maxRecoveryTime;
};

}  // namespace Navigation
