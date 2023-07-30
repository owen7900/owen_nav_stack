#pragma once

#include <owen_common/MailboxData.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "owen_navigation/mapping/obstacle_sources/BaseObstacleSource.hpp"

namespace Navigation::Mapping::ObstacleSources {
class LidarObstacleSource : public BaseObstacleSource {
  using Point2D = owen_common::types::Point2D;

 public:
  explicit LidarObstacleSource(rclcpp::Node& n);

  MapManager::MapT::MapUpdate GetMapUpdate(
      const owen_common::types::Pose2D& pose) override;
  bool HasMapUpdate() const override;

 private:
  owen_common::types::MailboxData<sensor_msgs::msg::LaserScan::SharedPtr> scan;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub;
  bool canClear;
  double raytraceResolution;
  double clearingRadius;
};
}  // namespace Navigation::Mapping::ObstacleSources
