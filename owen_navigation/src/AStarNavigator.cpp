#include "owen_navigation/path_generators/AStarNavigator.hpp"

namespace Navigation::PathGenerators {

namespace Constants {
const std::string Name = "AStar";
}  // namespace Constants

AStarNavigator::AStarNavigator(rclcpp::Node& node)
    : BasePathGenerator(node, Constants::Name) {
  destinationSub = node.create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 1, [this](const geometry_msgs::msg::PoseStamped& msg) {
        destination.SetData({msg.pose.position.x, msg.pose.position.y});
      });

  params.planningResolution =
      node.get_parameter_or("astar_planning_resolution", 0.1);
}

std::vector<owen_common::types::Point2D> AStarNavigator::GeneratePath() {
  std::vector<owen_common::types::Point2D> ret;

  return ret;
}

bool AStarNavigator::HasNewCommand() {
  const bool tmp = destination.HasNewData();
  destination.ResetFlag();
  return tmp;
}

bool AStarNavigator::HasUpdatedPath() const { return false; }

}  // namespace Navigation::PathGenerators
