#pragma once
#include <queue>

#include "owen_common/MailboxData.hpp"
#include "owen_common/Point2D.hpp"
#include "owen_navigation/path_generators/BaseNavigator.hpp"

namespace Navigation::PathGenerators {

struct Node {
  explicit Node(const owen_common::types::Point2D& point_ = {}, double c = 0,
                double h = 0)
      : point(point_), cost(c), heuristic(h), totalCost(c + h){};
  double getCost() const { return cost; }
  double getHeuristic() const { return heuristic; }
  const owen_common::types::Point2D& getPoint() const { return point; }
  double getTotalCost() const { return totalCost; };
  void setCost(double c) {
    cost = c;
    totalCost = cost + heuristic;
  }
  void setHeurisitc(double h) {
    heuristic = h;
    totalCost = cost + heuristic;
  }
  void setPoint(const owen_common::types::Point2D& p) { point = p; }

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

class AStarNavigator : public BaseNavigator {
  template <typename P>
  friend class PlannerTester;

 public:
  AStarNavigator(rclcpp::Node& node,
                 const std::shared_ptr<Mapping::MapManager>& map);

  std::vector<owen_common::types::Point2D> GeneratePath(
      const owen_common::types::Pose2D& pose) override;

  bool HasUpdatedPath() const override;

 private:
  void exploreAroundNode(const Node& n);
  void addNodeWithOffset(const Node& n,
                         const owen_common::types::Point2D& offset);
  bool isDestinationNode(const Node& n);

 private:
  std::unordered_map<Node, Node, NodeHash> prevNodes;
  std::unordered_set<Node, NodeHash> visitedNodes;

  std::priority_queue<Node, std::vector<Node>, std::greater<>> queue;
  Node startNode;
};

}  // namespace Navigation::PathGenerators
