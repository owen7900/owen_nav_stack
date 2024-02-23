#include "owen_navigation/path_generators/AStarNavigator.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/utilities.hpp>

#include "owen_common/AStar.hpp"

namespace Navigation::PathGenerators {

namespace Constants {
const std::string Name = "astar";
}  // namespace Constants

bool operator>(const Node& lhs, const Node& rhs) {
  return (lhs.getTotalCost()) > (rhs.getTotalCost());
}

bool operator!=(const Node& lhs, const Node& rhs) {
  return lhs.getPoint() != rhs.getPoint();
}

bool operator==(const Node& lhs, const Node& rhs) {
  return lhs.getPoint() == rhs.getPoint();
}
AStarNavigator::AStarNavigator(rclcpp::Node& node,
                               const std::shared_ptr<Mapping::MapManager>& map)
    : BaseNavigator(node, Constants::Name, map) {}

std::vector<owen_common::types::Point2D> AStarNavigator::GeneratePath(
    const owen_common::types::Pose2D& pose) {
  RCLCPP_INFO(rclcpp::get_logger("astar_navigator"), "Generating path");
  owen_common::AStar<Mapping::MapManager::MapT> astar;
  owen_common::AStar<Mapping::MapManager::MapT>::Params p{};
  p.planningTime = params.maxPlanningTime.count();
  p.planningResolution = params.planningResolution;
  p.vehicleRadius = params.vehicleRadius;
  const auto dest = destination.GetData();
  path = astar.PlanPath({pose.x, pose.y}, {dest.x, dest.y}, map->GetMap(), p);
  this->publishDebugPath(path);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("astar_navigator"),
                     "Generated path of size: " << path.size());
  return path;
}

bool AStarNavigator::HasUpdatedPath() const { return false; }

}  // namespace Navigation::PathGenerators
