#include "owen_navigation/mapping/obstacle_sources/ObstacleSourceFactory.hpp"

#include "owen_navigation/mapping/obstacle_sources/StaticMapObstacleSource.hpp"

namespace Navigation::Mapping::ObstacleSourceFactory {
std::unique_ptr<ObstacleSources::BaseObstacleSource> GetObstacleSource(
    const std::string& name, rclcpp::Node& node) {
  if (name == "static_map") {
    return std::make_unique<ObstacleSources::StaticMapObstacleSource>(node);
  }
  return nullptr;
}
}  // namespace Navigation::Mapping::ObstacleSourceFactory
