#pragma once
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <queue>

#include "owen_common/MailboxData.hpp"
#include "owen_navigation/path_generators/BasePathGenerator.hpp"

namespace Navigation::PathGenerators {

struct Node {
  owen_common::types::Point2D point;
  double cost;
  double heuristic;
  bool operator>(const Node& n) {
    return (this->cost + this->heuristic) > (n.cost + n.heuristic);
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

 private:
  owen_common::types::MailboxData<owen_common::types::Point2D> destination;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      destinationSub;

  struct Parameters {
    float planningResolution;
  };

  Parameters params;

  std::vector<owen_common::types::Point2D> path;
  std::priority_queue<Node, std::vector<Node>, std::greater<>> queue;
};

}  // namespace Navigation::PathGenerators
