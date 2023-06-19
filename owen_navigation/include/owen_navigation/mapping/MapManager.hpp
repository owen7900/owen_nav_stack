#pragma once

#include <rclcpp/node.hpp>

#include "owen_navigation/mapping/Map.hpp"
#include "owen_navigation/mapping/obstacle_sources/BaseObstacleSource.hpp"
namespace Navigation::Mapping {

class MapManager {
 public:
  explicit MapManager(rclcpp::Node& node);

  const Map& GetMap() const { return map; }

  void UpdateMap(const owen_common::types::Pose2D& pose);

 private:
  Map map;

  std::vector<std::unique_ptr<ObstacleSources::BaseObstacleSource>>
      obstacleSources;
};

}  // namespace Navigation::Mapping
