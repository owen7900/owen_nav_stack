#pragma once
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "owen_common/MailboxData.hpp"
#include "owen_navigation/path_generators/BasePathGenerator.hpp"

namespace Navigation::PathGenerators {

class AStarNavigator : public BasePathGenerator {
 public:
  AStarNavigator(rclcpp::Node& node,
                 const std::shared_ptr<Mapping::MapManager>& map);

  std::vector<owen_common::types::Point2D> GeneratePath(
      const owen_common::types::Pose2D& pose) override;

  bool HasNewCommand() override;

  bool HasUpdatedPath() const override;

 private:
  owen_common::types::MailboxData<owen_common::types::Point2D> destination;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      destinationSub;

  struct Parameters {
    float planningResolution;
  };

  Parameters params;
};

}  // namespace Navigation::PathGenerators
