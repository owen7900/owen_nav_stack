#include <owen_navigation/mapping/obstacle_sources/LidarObstacleSource.hpp>

namespace Navigation::Mapping::ObstacleSources {

namespace Constants {
const std::string Name = "lidar";
}  // namespace Constants

LidarObstacleSource::LidarObstacleSource(rclcpp::Node& n)
    : BaseObstacleSource(Constants::Name) {
  sub = n.create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 1, [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
        scan.SetData(std::move(msg));
      });

  canClear = n.get_parameter_or("lidar_map_clearing", true);
  raytraceResolution = n.get_parameter_or("lidar_map_raytrace_resolution", 0.1);
  clearingRadius =
      n.get_parameter_or("lidar_map_clearing_radius", raytraceResolution / 2.0);
}

bool LidarObstacleSource::HasMapUpdate() const { return scan.HasNewData(); }

MapManager::MapT::MapUpdate LidarObstacleSource::GetMapUpdate(
    const owen_common::types::Pose2D& pose) {
  const auto s = scan.GetData();
  MapManager::MapT::MapUpdate ret;

  const size_t numRanges = s->ranges.size();
  double angle = s->angle_min + pose.yaw;
  const double deltaAngle = (s->angle_max - s->angle_min) / numRanges;
  const Point2D centerPt{pose.x, pose.y};
  const Point2D clearing{clearingRadius, clearingRadius};

  for (size_t i = 0; i < numRanges; ++i, angle += deltaAngle) {
    const double sinA = std::sin(angle);
    const double cosA = std::cos(angle);
    const double range = s->ranges[i];
    const auto pt = centerPt + Point2D{range * sinA, range * cosA};

    if (canClear) {
      const double maxClearingRange = range - clearingRadius;
      for (double r = 0; r < maxClearingRange; r += raytraceResolution) {
        MapManager::MapT::Cell c;
        c.state = MapManager::MapT::Free;
        const auto center = centerPt + Point2D{r * sinA, r * cosA};
        c.bounds = {{center + clearing}, {center - clearing}};
        ret.push_back(c);
      }
    }

    MapManager::MapT::Cell c;
    c.state = MapManager::MapT::Occupied;
    c.bounds = MapManager::MapT::Rectangle{pt, pt};
    ret.push_back(c);
  }
  return ret;
}

}  // namespace Navigation::Mapping::ObstacleSources
