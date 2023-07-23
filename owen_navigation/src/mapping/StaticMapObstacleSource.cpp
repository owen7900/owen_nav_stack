#include "owen_navigation/mapping/obstacle_sources/StaticMapObstacleSource.hpp"

#include <rclcpp/logger.hpp>

namespace Navigation::Mapping::ObstacleSources {
using std::move;
using std::placeholders::_1;

namespace Constants {
constexpr double DefaultMapTimeout = 10.0;
const std::string Name = "static_map";
}  // namespace Constants

StaticMapObstacleSource::StaticMapObstacleSource(rclcpp::Node& n)
    : BaseObstacleSource(Constants::Name), gotNewMap(false) {
  mapSub = n.create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 1, [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        gotNewMap = true;
        map.SetData(msg);
      });

  mapTimeout = n.get_parameter_or("static_map_map_timeout",
                                  Constants::DefaultMapTimeout);
  canClear = n.get_parameter_or("static_map_clearing", false);
}

Map::MapUpdate StaticMapObstacleSource::GetMapUpdate(
    const owen_common::types::Pose2D& /*pose*/) {
  gotNewMap = false;
  if (map.GetDataAge() > mapTimeout) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger(Constants::Name), "Map is timed out");
    return {};
  }
  using Point = owen_common::types::Point2D;

  Map::MapUpdate update;
  const auto mapPtr = map.GetData();
  Point origin{mapPtr->info.origin.position.x, mapPtr->info.origin.position.y};

  size_t idx = 0;
  for (size_t y = 0; y < mapPtr->info.height; ++y) {
    for (size_t x = 0; x < mapPtr->info.width; ++x) {
      Map::Cell cell;
      cell.state = mapPtr->data[idx] > 0;
      cell.bounds = {origin + Point{x * mapPtr->info.resolution,
                                    y * mapPtr->info.resolution},
                     origin + Point{(x + 1) * mapPtr->info.resolution,
                                    (y + 1) * mapPtr->info.resolution}};
      if (mapPtr->data[idx++] >= 0 && (cell.state || canClear)) {
        update.push_back(cell);
      }
    }
  }

  return update;
}

bool StaticMapObstacleSource::HasMapUpdate() const { return this->gotNewMap; }

}  // namespace Navigation::Mapping::ObstacleSources
