#include "owen_navigation/mapping/obstacle_sources/ObstacleSourceFactory.hpp"

namespace Navigation::Mapping::ObstacleSourceFactory {
std::unique_ptr<ObstacleSources::BaseObstacleSource> GetObstacleSource(
    const std::string& name, rclcpp::Node& node) {
  return nullptr;
}
}  // namespace Navigation::Mapping::ObstacleSourceFactory
