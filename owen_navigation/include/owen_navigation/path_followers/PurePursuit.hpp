#pragma once

#include "owen_navigation/path_followers/BasePathFollower.hpp"
namespace Navigation::PathFollowers {

class PurePursuit : public BasePathFollower {
 public:
  explicit PurePursuit(rclcpp::Node& n,
                       const std::shared_ptr<Mapping::MapManager>& map);

  std::optional<BasePathFollower::Command> CalculateCommand(
      const owen_common::types::Pose2D& pose) override;
};

}  // namespace Navigation::PathFollowers
