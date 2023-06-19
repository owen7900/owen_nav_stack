#include "owen_navigation/path_followers/PurePursuit.hpp"

namespace Navigation::PathFollowers {

PurePursuit::PurePursuit(rclcpp::Node& n,
                         const std::shared_ptr<Mapping::MapManager>& map)
    : BasePathFollower(map) {}

std::optional<BasePathFollower::Command> PurePursuit::CalculateCommand(
    const owen_common::types::Pose2D& pose) {
  return {};
}

}  // namespace Navigation::PathFollowers
