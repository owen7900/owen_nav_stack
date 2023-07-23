#include "owen_navigation/path_generators/AStarNavigator.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/utilities.hpp>

namespace Navigation::PathGenerators {

namespace Constants {
const std::string Name = "AStar";
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
  queue = std::priority_queue<Node, std::vector<Node>, std::greater<>>();
  this->prevNodes.clear();
  visitedNodes.clear();
  startNode =
      Node{{pose.x, pose.y},
           0.0,
           destination.GetDataRef().distanceFromPoint({pose.x, pose.y})};
  queue.push(startNode);

  while (!queue.empty() && !this->isDestinationNode(queue.top()) &&
         rclcpp::ok()) {
    const auto top = queue.top();
    queue.pop();
    if (visitedNodes.count(top) <= 0) {
      visitedNodes.insert(top);
      this->exploreAroundNode(top);
    }
  }

  visitedNodes.clear();
  std::cout << "Done exploring: " << std::endl;

  std::vector<owen_common::types::Point2D> ret;

  Node n = queue.top();
  Node prv;
  while (n != startNode && rclcpp::ok()) {
    if (visitedNodes.count(n) > 0) {
      std::cout << "GOT circle in prev " << n.getPoint()
                << " P: " << prv.getPoint() << std::endl;
      break;
    }
    visitedNodes.insert(n);
    ret.push_back(n.getPoint());
    prv = n;
    n = this->prevNodes[n];
  }
  ret.push_back(startNode.getPoint());
  std::reverse(ret.begin(), ret.end());
  this->publishDebugPath(ret);
  RCLCPP_INFO_STREAM(rclcpp::get_logger(Constants::Name),
                     "Explored: " << this->prevNodes.size() << " nodes");
  return ret;
}

bool AStarNavigator::isDestinationNode(const Node& n) {
  return std::abs(n.getPoint().x - destination.GetDataRef().x) <=
             params.planningResolution &&
         std::abs(n.getPoint().y - destination.GetDataRef().y) <=
             params.planningResolution;
}

bool AStarNavigator::HasUpdatedPath() const { return false; }

void AStarNavigator::exploreAroundNode(const Node& n) {
  addNodeWithOffset(n, {params.planningResolution, 0.0});
  addNodeWithOffset(n, {-params.planningResolution, 0.0});
  addNodeWithOffset(n, {0.0, params.planningResolution});
  addNodeWithOffset(n, {0.0, -params.planningResolution});

  addNodeWithOffset(n, {params.planningResolution, params.planningResolution});
  addNodeWithOffset(n, {-params.planningResolution, params.planningResolution});
  addNodeWithOffset(n, {params.planningResolution, -params.planningResolution});
  addNodeWithOffset(n,
                    {-params.planningResolution, -params.planningResolution});

  addNodeWithOffset(n,
                    {2 * params.planningResolution, params.planningResolution});
  addNodeWithOffset(
      n, {2 * params.planningResolution, -params.planningResolution});
  addNodeWithOffset(
      n, {-2 * params.planningResolution, params.planningResolution});
  addNodeWithOffset(
      n, {-2 * params.planningResolution, -params.planningResolution});
  addNodeWithOffset(n,
                    {params.planningResolution, 2 * params.planningResolution});
  addNodeWithOffset(
      n, {-params.planningResolution, 2 * params.planningResolution});
  addNodeWithOffset(
      n, {params.planningResolution, -2 * params.planningResolution});
  addNodeWithOffset(
      n, {-params.planningResolution, -2 * params.planningResolution});
}

void AStarNavigator::addNodeWithOffset(
    const Node& n, const owen_common::types::Point2D& offset) {
  Node newNode{n.getPoint() + offset};
  if (!map->GetMap().IsSafePath(n.getPoint(), newNode.getPoint(),
                                params.vehicleRadius)) {
    return;
  }
  newNode.setHeurisitc(
      newNode.getPoint().distanceFromPoint(destination.GetDataRef()));
  newNode.setCost(newNode.getPoint().distanceFromPoint(n.getPoint()) +
                  n.getCost());
  if (prevNodes.count(newNode) > 0) {
    if (prevNodes[newNode].getCost() > newNode.getCost()) {
      prevNodes[newNode] = n;
      prevNodes[newNode].setCost(newNode.getCost());
      queue.push(newNode);
    }
  } else {
    prevNodes[newNode] = n;
    prevNodes[newNode].setCost(newNode.getCost());
    queue.push(newNode);
  }
}

}  // namespace Navigation::PathGenerators
