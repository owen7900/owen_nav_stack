#pragma once
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <queue>

#include "owen_common/MailboxData.hpp"
#include "owen_common/Point2D.hpp"
#include "owen_navigation/path_generators/BasePathGenerator.hpp"

namespace Navigation::PathGenerators {

struct Node {
  owen_common::types::Point2D point;
  double cost;
  double heuristic;
};

struct NodeHash {
  size_t operator()(const Node& n) const {
    return owen_common::types::BasePointHash{}(n.point);
  }
};

class AStarNavigator : public BasePathGenerator {
 public:
  AStarNavigator(rclcpp::Node& node,
                 const std::shared_ptr<Mapping::MapManager>& map);

  std::vector<owen_common::types::Point2D> GeneratePath(
      const owen_common::types::Pose2D& pose) override;

  bool HasNewCommand() override;

  bool HasUpdatedPath() const override;

 private:
  void exploreAroundNode(const Node& n);
  void addNodeWithOffset(const Node& n,
                         const owen_common::types::Point2D& offset);
  bool isDestinationNode(const Node& n);

  void publishDebugPath(
      const std::vector<owen_common::types::Point2D>& path) const;

 private:
  owen_common::types::MailboxData<owen_common::types::Point2D> destination;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      destinationSub;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr debugPathPub;

  std::unordered_map<Node, Node, NodeHash> prevNodes;

  struct Parameters {
    float planningResolution;
  };

  Parameters params;

  std::vector<owen_common::types::Point2D> path;
  std::priority_queue<Node, std::vector<Node>, std::greater<>> queue;
};

}  // namespace Navigation::PathGenerators
