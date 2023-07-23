#include "owen_navigation/path_generators/RRTNavigator.hpp"

namespace Navigation::PathGenerators {

namespace Constants {
const std::string Name = "rrt";
const size_t NumSteps = 1000;
}  // namespace Constants

bool operator!=(const RRTNavigator::Node& lhs, const RRTNavigator::Node& rhs) {
  return lhs.point != rhs.point;
}

RRTNavigator::RRTNavigator(rclcpp::Node& node,
                           const std::shared_ptr<Mapping::MapManager>& map)
    : BaseNavigator(node, Constants::Name, map) {
  rrtParams.successRadius =
      node.get_parameter_or(Constants::Name + "_success_radius", 1.0);
  rrtParams.exploreDistance =
      node.get_parameter_or(Constants::Name + "_explore_distance", 10.0);
  rrtParams.exploreFraction = rrtParams.exploreDistance / Constants::NumSteps;
}

std::vector<owen_common::types::Point2D> RRTNavigator::GeneratePath(
    const owen_common::types::Pose2D& pose) {
  nodes.clear();

  Node first;
  first.point = {pose.x, pose.y};
  nodes.push_back(first);

  while (rclcpp::ok()) {
    const size_t idx = rand() % nodes.size();
    if (idx < nodes.size()) {
      const auto pt = randomlySampleAround(nodes.at(idx));
      addNode(pt, idx);
      if (isPointDestination(pt)) {
        break;
      }
    } else {
      RCLCPP_INFO_STREAM(
          rclcpp::get_logger("rrt"),
          "Got unknow idx: " << idx << " for num nodes: " << nodes.size());
    }
  }

  RCLCPP_INFO_STREAM(rclcpp::get_logger("rrt"),
                     "Explored: " << nodes.size() << "nodes");
  std::vector<owen_common::types::Point2D> ret;
  Node n = nodes.back();
  ret.push_back(n.point);
  while (n != first) {
    n = nodes.at(n.parent);
    ret.push_back(n.point);
  }

  std::reverse(ret.begin(), ret.end());
  return ret;
}

owen_common::types::Point2D RRTNavigator::randomlySampleAround(
    const Node& n) const {
  auto ret = n.point;

  while (rclcpp::ok()) {
    ret = ret +
          Point2D{rrtParams.exploreFraction * (rand() % Constants::NumSteps),
                  rrtParams.exploreFraction * (rand() % Constants::NumSteps)};
    if (map->GetMap().IsSafePath(ret, n.point, params.vehicleRadius)) {
      break;
    }
  }

  return ret;
}

bool RRTNavigator::HasUpdatedPath() const { return false; }

void RRTNavigator::addNode(const owen_common::types::Point2D& pt,
                           size_t parent) {
  if (parent >= nodes.size()) {
    return;
  }
  Node n;
  n.point = pt;
  n.parent = parent;
  nodes.at(parent).children.push_back(parent);
  nodes.push_back(n);
}

bool RRTNavigator::isPointDestination(const Point2D& pt) const {
  return pt.distanceFromPoint(destination.PeekData()) < rrtParams.successRadius;
}

}  // namespace Navigation::PathGenerators
