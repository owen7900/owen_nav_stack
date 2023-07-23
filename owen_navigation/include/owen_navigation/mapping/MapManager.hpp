#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/node.hpp>

#include "owen_navigation/mapping/Map.hpp"
#include "owen_navigation/mapping/obstacle_sources/BaseObstacleSource.hpp"
namespace Navigation::Mapping {

class MapManager {
 public:
  explicit MapManager(rclcpp::Node& node);

  const Map& GetMap() const { return map; }
  Map& GetMapRef() { return map; }

  void UpdateMap(const owen_common::types::Pose2D& pose);

 private:
  nav_msgs::msg::OccupancyGrid generateNavMap() const;

 private:
  Map map;

  std::vector<std::unique_ptr<ObstacleSources::BaseObstacleSource>>
      obstacleSources;

  rclcpp::TimerBase::SharedPtr publishMapTimer;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPub;
};

}  // namespace Navigation::Mapping
