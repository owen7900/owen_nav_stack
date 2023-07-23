#include "owen_navigation/path_generators/RRTNavigator.hpp"

namespace Navigation::PathGenerators {

namespace Constants {
const std::string Name = "rrt";
const size_t NumSteps = 10000;
}  // namespace Constants

bool operator!=(const RRTNavigator::Node& lhs, const RRTNavigator::Node& rhs) {
  return lhs.point != rhs.point;
}

RRTNavigator::RRTNavigator(rclcpp::Node& node,
                           const std::shared_ptr<Mapping::MapManager>& map)
    : BaseNavigator(node, Constants::Name, map) {
  rrtParams.successRadius = node.get_parameter_or(
      Constants::Name + "_success_radius", params.planningResolution);
  rrtParams.exploreDistance =
      node.get_parameter_or(Constants::Name + "_explore_distance", 10.0);
  rrtParams.exploreFraction = rrtParams.exploreDistance / Constants::NumSteps;
}

std::vector<owen_common::types::Point2D> RRTNavigator::GeneratePath(
    const owen_common::types::Pose2D& pose) {
  nodes.clear();
  map->GetMapRef().ResizeToPoint({pose.x, pose.y});
  map->GetMapRef().ResizeToPoint(destination.PeekData());
  if (!map->GetMap().IsSafe(destination.PeekData(), params.vehicleRadius) ||
      !map->GetMap().IsSafe({pose.x, pose.y})) {
    RCLCPP_ERROR(rclcpp::get_logger("rrt"), "Destination or start is not safe");
    return {};
  }

  Node first;
  first.point = {pose.x, pose.y};
  nodes.push_back(first);
  if (isPointDestination(first.point)) {
    return {first.point, destination.PeekData()};
  }

  while (rclcpp::ok()) {
    const size_t idx = rand() % nodes.size();
    if (idx < nodes.size()) {
      const auto pt = randomlySamplePoint();
      if (isPointDestination(pt)) {
        addNode(destination.PeekData(), nodes.size() - 1);
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

owen_common::types::Point2D RRTNavigator::randomlySamplePoint() {
  Point2D ret;
  const auto minPt = map->GetMap().GetOrigin() - Point2D{-10, -10};
  const auto maxPt = map->GetMap().GetMaxPoint();
  const auto diff = maxPt - minPt;
  const auto width = diff.x + 10;
  const auto height = diff.y + 10;
  const int numHeightSteps = width / params.planningResolution;
  const int numWidthSteps = height / params.planningResolution;

  while (rclcpp::ok()) {
    ret =
        minPt + Point2D{params.planningResolution * (rand() % numWidthSteps),
                        params.planningResolution * (rand() % numHeightSteps)};
    double minDist = std::numeric_limits<double>::max();
    size_t closest = 0;
    bool haveOne = false;
    const size_t numPts = nodes.size();
    for (size_t i = 0; i < numPts; ++i) {
      const auto& pt = nodes[i];
      const double dist = ret.distanceFromPoint(pt.point);
      if (dist < minDist &&
          map->GetMap().IsSafePath(ret, pt.point, params.vehicleRadius)) {
        minDist = dist;
        haveOne = true;
        closest = i;
      }
    }
    if (haveOne) {
      addNode(ret, closest);
      break;
    }
  }
  // std::cout << "Sampled: " << ret << " numSteps: " << numWidthSteps << ", "
  //           << numHeightSteps << " res: " << params.planningResolution
  //           << " rad: " << params.vehicleRadius << " width: " << width
  //           << " height:" << height << " minPt: " << minPt
  //           << " maxPt: " << maxPt << std::endl;

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
  return map->GetMap().IsSafePath(pt, destination.PeekData(),
                                  params.vehicleRadius);
}

}  // namespace Navigation::PathGenerators
