#include "owen_navigation/path_generators/AStarNavigator.hpp"

namespace Navigation::PathGenerators {

namespace Constants {
const std::string Name = "AStar";
}  // namespace Constants

AStarNavigator::AStarNavigator(rclcpp::Node& node,
                               const std::shared_ptr<Mapping::MapManager>& map)
    : BasePathGenerator(node, Constants::Name, map) {
  destinationSub = node.create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 1, [this](const geometry_msgs::msg::PoseStamped& msg) {
        destination.SetData({msg.pose.position.x, msg.pose.position.y});
      });

  params.planningResolution =
      node.get_parameter_or("astar_planning_resolution", 0.1);
}

std::vector<owen_common::types::Point2D> AStarNavigator::GeneratePath(
    const owen_common::types::Pose2D& pose) {
  queue = std::priority_queue<Node, std::vector<Node>, std::greater<>>();
  queue.push({{pose.x, pose.y},
              0.0,
              destination.GetDataRef().distanceFromPoint({pose.x, pose.y})});

  while (!queue.empty() && !this->isDestinationNode(queue.top())) {
    const auto top = queue.top();
    queue.pop();
    this->exploreAroundNode(top);
  }

  std::vector<owen_common::types::Point2D> ret;
  return ret;
}

bool AStarNavigator::HasNewCommand() {
  const bool tmp = destination.HasNewData();
  destination.ResetFlag();
  return tmp;
}

bool AStarNavigator::isDestinationNode(const Node& n) {
  return std::abs(n.point.x - destination.GetDataRef().x) <=
             params.planningResolution &&
         std::abs(n.point.y - destination.GetDataRef().y) <=
             params.planningResolution;
}

bool AStarNavigator::HasUpdatedPath() const { return false; }

void AStarNavigator::exploreAroundNode(const Node& n) {
  addNodeWithOffset(n, {params.planningResolution, 0.0});
  addNodeWithOffset(n, {-params.planningResolution, 0.0});
  addNodeWithOffset(n, {0.0, params.planningResolution});
  addNodeWithOffset(n, {0.0, -params.planningResolution});
}

void AStarNavigator::addNodeWithOffset(
    const Node& n, const owen_common::types::Point2D& offset) {
  Node newNode = n;
  newNode.point = newNode.point + offset;
  if (map->GetMap().IsOccupied(newNode.point)) {
    return;
  }
  newNode.heuristic = newNode.point.distanceFromPoint(destination.GetDataRef());
  queue.push(newNode);
  newNode.cost += newNode.point.distanceFromPoint(n.point);
}

}  // namespace Navigation::PathGenerators
