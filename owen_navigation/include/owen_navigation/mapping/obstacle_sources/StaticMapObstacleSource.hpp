#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <owen_common/TimestampData.hpp>
#include <rclcpp/node.hpp>

#include "owen_navigation/mapping/obstacle_sources/BaseObstacleSource.hpp"
namespace Navigation::Mapping::ObstacleSources {
class StaticMapObstacleSource : public BaseObstacleSource {
 public:
  explicit StaticMapObstacleSource(rclcpp::Node& n);

  MapManager::MapT::MapUpdate GetMapUpdate(
      const owen_common::types::Pose2D& /*pose*/) override;

  bool HasMapUpdate() const override;

 private:
  owen_common::TimeoutData<nav_msgs::msg::OccupancyGrid::SharedPtr> map;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSub;
  bool gotNewMap;

  double mapTimeout;
  bool canClear;
};

}  // namespace Navigation::Mapping::ObstacleSources
