#include "owen_navigation/mapping/obstacle_sources/StaticMapObstacleSource.hpp"

namespace Navigation::Mapping::ObstacleSources {
using std::move;
using std::placeholders::_1;

namespace Constants {
constexpr double DefaultMapTimeout = 10.0;
}  // namespace Constants

StaticMapObstacleSource::StaticMapObstacleSource(rclcpp::Node& n) {
  mapSub = n.create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 1,
      [this](const nav_msgs::msg::OccupancyGrid& msg) { map.SetData(msg); });

  mapTimeout = n.get_parameter_or("static_map_map_timeout",
                                  Constants::DefaultMapTimeout);
}

Map::MapUpdate StaticMapObstacleSource::GetMapUpdate(
    const owen_common::types::Pose2D& /*pose*/) const {
  if (map.GetDataAge() > mapTimeout) {
    return {};
  }
  using Point = owen_common::types::Point2D;

  Map::MapUpdate update;
  const auto& mapRef = map.GetDataRef();
  Point origin{mapRef.info.origin.position.x, mapRef.info.origin.position.y};

  size_t idx = 0;
  for (size_t y = 0; y < mapRef.info.height; ++y) {
    for (size_t x = 0; x < mapRef.info.width; ++x) {
      Map::Cell cell;
      cell.state = mapRef.data[idx++] > 0;
      cell.bounds = {origin + Point{x * mapRef.info.resolution,
                                    y * mapRef.info.resolution},
                     origin + Point{(x + 1) * mapRef.info.resolution,
                                    (y + 1) * mapRef.info.resolution}};
      update.push_back(cell);
    }
  }

  return update;
}

}  // namespace Navigation::Mapping::ObstacleSources
