#include "owen_navigation/path_generators/RRTNavigator.hpp"

namespace Navigation::PathGenerators {

namespace Constants {
const std::string Name = "rrt";
}  // namespace Constants

bool operator!=(const RRTNavigator::Node& lhs, const RRTNavigator::Node& rhs) {
  return lhs.point != rhs.point;
}

RRTNavigator::RRTNavigator(rclcpp::Node& node,
                           const std::shared_ptr<Mapping::MapManager>& map)
    : BaseNavigator(node, Constants::Name, map) {
  rrtParams.doOptimize =
      node.get_parameter_or(Constants::Name + "_optimize", true);
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
  first.cost = 0;
  nodes.push_back(first);
  if (isPointDestination(first.point)) {
    return {first.point, destination.PeekData()};
  }
  bool foundPath = false;
  const auto startTime = std::chrono::system_clock::now();
  size_t closestPt = 0;
  double minCost = std::numeric_limits<double>::max();

  while (rclcpp::ok()) {
    const auto pt = randomlySamplePoint();

    const double cost =
        destination.PeekData().distanceFromPoint(pt) + nodes.back().cost;
    if (cost < minCost && isPointDestination(pt)) {
      closestPt = nodes.size() - 1;
      minCost = cost;
      foundPath = true;
    }
    if ((params.maxPlanningTime <
             std::chrono::system_clock::now() - startTime &&
         foundPath) ||
        (!rrtParams.doOptimize && foundPath)) {
      break;
    }
  }

  RCLCPP_INFO_STREAM(rclcpp::get_logger("rrt"),
                     "Explored: " << nodes.size() << "nodes");
  std::vector<owen_common::types::Point2D> ret;
  ret.push_back(destination.PeekData());
  Node n = nodes[closestPt];
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
    std::vector<std::pair<bool, bool>> safeNodes(numPts, {false, false});
    for (size_t i = 0; i < numPts; ++i) {
      const auto& pt = nodes[i];
      const double dist = ret.distanceFromPoint(pt.point) + pt.cost;
      if (dist < minDist) {
        const bool isSafe =
            map->GetMap().IsSafePath(ret, pt.point, params.vehicleRadius);
        if (isSafe) {
          minDist = dist;
          haveOne = true;
          closest = i;
        }
        safeNodes[i] = {isSafe, true};
      }
    }
    if (haveOne) {
      addNode(ret, closest, minDist);
      for (size_t i = 0; i < numPts; ++i) {
        auto& node = nodes[i];
        const double newCost = minDist + node.point.distanceFromPoint(ret);
        if (node.cost > newCost) {
          bool isSafe = safeNodes[i].first;
          if (!safeNodes[i].second) {
            isSafe =
                map->GetMap().IsSafePath(ret, node.point, params.vehicleRadius);
          }
          if (isSafe) {
            auto& c = nodes[node.parent].children;
            c.erase(std::remove(c.begin(), c.end(), i), c.end());
            node.parent = nodes.size() - 1;
            nodes.back().children.push_back(i);
          }
        }
      }
      break;
    }
  }
  return ret;
}

bool RRTNavigator::HasUpdatedPath() const { return false; }

void RRTNavigator::addNode(const owen_common::types::Point2D& pt, size_t parent,
                           double cost) {
  if (parent >= nodes.size()) {
    return;
  }
  Node n;
  n.point = pt;
  n.parent = parent;
  n.cost = cost;
  nodes.at(parent).children.push_back(parent);
  nodes.push_back(n);
}

bool RRTNavigator::isPointDestination(const Point2D& pt) const {
  return map->GetMap().IsSafePath(pt, destination.PeekData(),
                                  params.vehicleRadius);
}

}  // namespace Navigation::PathGenerators
