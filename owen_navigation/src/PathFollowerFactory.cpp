#include "owen_navigation/path_followers/PathFollowerFactory.hpp"

#include "owen_navigation/path_followers/PurePursuit.hpp"
namespace Navigation::PathFollowers::PathFollowerFactory {
std::unique_ptr<BasePathFollower> GetFollower(
    rclcpp::Node& n, const std::shared_ptr<Mapping::MapManager>& map,
    const std::string& name) {
  if (name == "pure_pursuit") {
    return std::make_unique<PurePursuit>(n, map);
  }

  return nullptr;
}
}  // namespace Navigation::PathFollowers::PathFollowerFactory
