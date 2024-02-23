#pragma once

#include <chrono>
#include <queue>
#include <unordered_set>

#include "owen_common/Point2D.hpp"
namespace owen_common {

struct Node {
  explicit Node(const owen_common::types::Point2D& point_ = {}, double c = 0,
                double h = 0)
      : point(point_), cost(c), heuristic(h), totalCost(c + h){};
  [[nodiscard]] double getCost() const { return cost; }
  [[nodiscard]] double getHeuristic() const { return heuristic; }
  [[nodiscard]] const owen_common::types::Point2D& getPoint() const {
    return point;
  }
  [[nodiscard]] double getTotalCost() const { return totalCost; };
  void setCost(double c) {
    cost = c;
    totalCost = cost + heuristic;
  }
  void setHeurisitc(double h) {
    heuristic = h;
    totalCost = cost + heuristic;
  }
  void setPoint(const owen_common::types::Point2D& p) { point = p; }

  friend bool operator>(const Node& lhs, const Node& rhs) {
    return (lhs.getTotalCost()) > (rhs.getTotalCost());
  }

  friend bool operator!=(const Node& lhs, const Node& rhs) {
    return lhs.getPoint() != rhs.getPoint();
  }

  friend bool operator==(const Node& lhs, const Node& rhs) {
    return lhs.getPoint() == rhs.getPoint();
  }

 private:
  owen_common::types::Point2D point;
  double cost;
  double heuristic;
  double totalCost;
};

struct NodeHash {
  size_t operator()(const Node& n) const {
    return owen_common::types::BasePointHash{}(n.getPoint());
  }
};

template <typename MapT>
class AStar {
 public:
  struct Params {
    double planningResolution;
    double planningTime;
    double vehicleRadius;
  };

 public:
  AStar() = default;
  AStar(const AStar&) = default;
  AStar(AStar&&) = default;
  AStar& operator=(const AStar&) = default;
  AStar& operator=(AStar&&) = default;
  ~AStar() = default;

  std::vector<owen_common::types::Point2D> PlanPath(
      const owen_common::types::Point2D& start,
      const owen_common::types::Point2D& end, const MapT& map,
      const Params& p) {
    params = p;

    queue = std::priority_queue<Node, std::vector<Node>, std::greater<>>();
    this->prevNodes.clear();
    visitedNodes.clear();
    startNode = Node{start, 0.0, end.distanceFromPoint(start)};
    endNode = Node{end, std::numeric_limits<double>::max(), 0.0};
    queue.push(startNode);

    const auto startTime = std::chrono::system_clock::now();
    Node closest;
    double minDist = std::numeric_limits<double>::max();

    while (!queue.empty() && !this->isDestinationNode(queue.top()) &&
           std::chrono::duration_cast<std::chrono::duration<double>>(
               std::chrono::system_clock::now() - startTime)
                   .count() < params.planningTime) {
      const auto top = queue.top();
      queue.pop();
      if (visitedNodes.count(top) <= 0) {
        visitedNodes.insert(top);
        const double dist = top.getPoint().distanceFromPoint(end);
        if (dist < minDist) {
          closest = top;
          minDist = dist;
        }
        this->exploreAroundNode(top, map);
      }
    }
    if (std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::system_clock::now() - startTime)
            .count() > params.planningTime) {
      RCLCPP_WARN(rclcpp::get_logger("astar"), "Ran out of planning time");
    }

    visitedNodes.clear();

    std::vector<owen_common::types::Point2D> ret;

    Node n = queue.top();
    if (!isDestinationNode(n)) {
      n = closest;
    }
    Node prv;
    while (n != startNode) {
      if (visitedNodes.count(n) > 0) {
        break;
      }
      visitedNodes.insert(n);
      ret.push_back(n.getPoint());
      prv = n;
      n = this->prevNodes[n];
    }
    ret.push_back(startNode.getPoint());
    std::reverse(ret.begin(), ret.end());
    return ret;
  }

 private:
  void exploreAroundNode(const Node& n, const MapT& map) {
    addNodeWithOffset(n, {params.planningResolution, 0.0}, map);
    addNodeWithOffset(n, {-params.planningResolution, 0.0}, map);
    addNodeWithOffset(n, {0.0, params.planningResolution}, map);
    addNodeWithOffset(n, {0.0, -params.planningResolution}, map);

    addNodeWithOffset(n, {params.planningResolution, params.planningResolution},
                      map);
    addNodeWithOffset(
        n, {-params.planningResolution, params.planningResolution}, map);
    addNodeWithOffset(
        n, {params.planningResolution, -params.planningResolution}, map);
    addNodeWithOffset(
        n, {-params.planningResolution, -params.planningResolution}, map);

    addNodeWithOffset(
        n, {2 * params.planningResolution, params.planningResolution}, map);
    addNodeWithOffset(
        n, {2 * params.planningResolution, -params.planningResolution}, map);
    addNodeWithOffset(
        n, {-2 * params.planningResolution, params.planningResolution}, map);
    addNodeWithOffset(
        n, {-2 * params.planningResolution, -params.planningResolution}, map);
    addNodeWithOffset(
        n, {params.planningResolution, 2 * params.planningResolution}, map);
    addNodeWithOffset(
        n, {-params.planningResolution, 2 * params.planningResolution}, map);
    addNodeWithOffset(
        n, {params.planningResolution, -2 * params.planningResolution}, map);
    addNodeWithOffset(
        n, {-params.planningResolution, -2 * params.planningResolution}, map);
  };
  void addNodeWithOffset(const Node& n,
                         const owen_common::types::Point2D& offset,
                         const MapT& map) {
    Node newNode{n.getPoint() + offset};
    if (!map.IsSafePath(n.getPoint(), newNode.getPoint(),
                        params.vehicleRadius)) {
      return;
    }
    newNode.setHeurisitc(
        newNode.getPoint().distanceFromPoint(endNode.getPoint()));
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
  bool isDestinationNode(const Node& n) {
    return std::abs(n.getPoint().x - endNode.getPoint().x) <=
               params.planningResolution &&
           std::abs(n.getPoint().y - endNode.getPoint().y) <=
               params.planningResolution;
  }

 private:
  std::unordered_map<Node, Node, NodeHash> prevNodes;
  std::unordered_set<Node, NodeHash> visitedNodes;
  std::priority_queue<Node, std::vector<Node>, std::greater<>> queue;

  Node startNode;
  Node endNode;
  Params params;
};

}  // namespace owen_common
