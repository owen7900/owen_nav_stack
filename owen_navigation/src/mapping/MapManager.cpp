#include "owen_navigation/mapping/MapManager.hpp"

#include <tf2/LinearMath/Quaternion.h>

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

  mapPub = node.create_publisher<nav_msgs::msg::OccupancyGrid>("debug_map", 1);
  publishMapTimer =
      node.create_wall_timer(std::chrono::duration<double>(1.0),
                             [this]() { mapPub->publish(generateNavMap()); });
}

void MapManager::UpdateMap(const owen_common::types::Pose2D& pose) {
  Map::MapUpdate update;
  for (const auto& source : this->obstacleSources) {
    const auto mapUpdates = source->GetMapUpdate(pose);
    update.insert(update.end(), mapUpdates.begin(), mapUpdates.end());
  }

  this->map.UpdateMap(update);
}

nav_msgs::msg::OccupancyGrid MapManager::generateNavMap() const {
  nav_msgs::msg::OccupancyGrid g;
  g.header.frame_id = "map";

  g.info.width = map.GetWidth();
  g.info.height = map.GetHeight();
  g.info.resolution = map.GetResolution();

  auto origin = map.GetOrigin();
  g.info.origin.position.x = origin.x;
  g.info.origin.position.y = origin.y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  g.info.origin.orientation.x = q.x();
  g.info.origin.orientation.y = q.y();
  g.info.origin.orientation.z = q.z();
  g.info.origin.orientation.w = q.w();

  g.data.assign(map.GetData().begin(), map.GetData().end());

  return g;
}

}  // namespace Navigation::Mapping
