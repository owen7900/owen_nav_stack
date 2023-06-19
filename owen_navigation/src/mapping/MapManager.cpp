#include "owen_navigation/mapping/MapManager.hpp"

#include "owen_navigation/mapping/obstacle_sources/ObstacleSourceFactory.hpp"

namespace Navigation::Mapping {

namespace Constants {
constexpr double DefaultResolution = 0.1;
constexpr double DefaultMapSize = 10.0;
}  // namespace Constants

MapManager::MapManager(rclcpp::Node& node) {
  auto obstacleSourcesStrings = node.get_parameter_or(
      "obstacle_sources", std::vector<std::string>{"static_map"});

  const double mapSize =
      node.get_parameter_or("map_size", Constants::DefaultMapSize);
  const double mapResolution =
      node.get_parameter_or("map_resolution", Constants::DefaultResolution);
  map = Map({-mapSize, -mapSize}, {mapSize, mapSize}, mapResolution);

  for (const auto& s : obstacleSourcesStrings) {
    auto source = ObstacleSourceFactory::GetObstacleSource(s, node);
    if (source) {
      this->obstacleSources.push_back(std::move(source));
    }
  }
}

void MapManager::UpdateMap() {
  Map::MapUpdate update;
  for (const auto& source : this->obstacleSources) {
    const auto mapUpdates = source->GetMapUpdate();
    update.insert(update.end(), mapUpdates.begin(), mapUpdates.end());
  }

  this->map.UpdateMap(update);
}

}  // namespace Navigation::Mapping
