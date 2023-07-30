#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <owen_common/Pose2D.hpp>
#include <rclcpp/node.hpp>

#include "owen_navigation/mapping/Map.hpp"

namespace Navigation::Mapping {
namespace ObstacleSources {
class BaseObstacleSource;
}  // namespace ObstacleSources

class MapManager {
 public:
  using MapT = Map<uint8_t>;
  explicit MapManager(rclcpp::Node& node);

  const MapT& GetMap() const { return map; }
  MapT& GetMapRef() { return map; }

  void UpdateMap(const owen_common::types::Pose2D& pose);

 private:
  nav_msgs::msg::OccupancyGrid generateNavMap() const;

 private:
  MapT map;

  std::vector<std::unique_ptr<ObstacleSources::BaseObstacleSource>>
      obstacleSources;

  rclcpp::TimerBase::SharedPtr publishMapTimer;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPub;
};

}  // namespace Navigation::Mapping
