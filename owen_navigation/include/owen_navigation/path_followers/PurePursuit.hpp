#pragma once

#include "owen_navigation/path_followers/BasePathFollower.hpp"
namespace Navigation::PathFollowers {

class PurePursuit : public BasePathFollower {
 public:
  explicit PurePursuit(rclcpp::Node& n,
                       const std::shared_ptr<Mapping::MapManager>& map);

  std::optional<BasePathFollower::Command> CalculateCommand(
      const owen_common::types::Pose2D& pose) override;

 private:
  size_t getTargetIdx() const;
  size_t getClosestPointAlongPath() const;

  rcl_interfaces::msg::SetParametersResult updateParams(
      const std::vector<rclcpp::Parameter>& params);

  void setDefaultParamValues(rclcpp::Node& n);

 private:
  struct Params {
    double lookaheadDistance;
    double successRadius;
    double turnGain;
    double minObstacleDistance;
    double obstacleTurnLimit;
    double forwardGain;
    double angularLinearWeight;
    double maxForwardVel;
  };
  Params params;

  owen_common::types::Pose2D pose;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      paramsCallbackHandle;
};

}  // namespace Navigation::PathFollowers
