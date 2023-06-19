#pragma once
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "owen_common/MailboxData.hpp"
#include "owen_navigation/path_generators/BasePathGenerator.hpp"

namespace Navigation::PathGenerators {

class AStarNavigator : public BasePathGenerator {
 public:
  explicit AStarNavigator(rclcpp::Node& node);

  std::vector<owen_common::types::Point2D> GeneratePath() override;

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
