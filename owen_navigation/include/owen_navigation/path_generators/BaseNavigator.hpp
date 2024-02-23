
#pragma once
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include "owen_common/MailboxData.hpp"
#include "owen_common/Point2D.hpp"
#include "owen_navigation/mapping/MapManager.hpp"
#include "owen_navigation/path_generators/BasePathGenerator.hpp"
namespace Navigation::PathGenerators {

class BaseNavigator : public BasePathGenerator {
 public:
  BaseNavigator(rclcpp::Node& node, const std::string& name,
                const std::shared_ptr<Mapping::MapManager>& map)
      : BasePathGenerator(node, name, map) {
    destinationSub = node.create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 1, [this](const geometry_msgs::msg::PoseStamped& msg) {
          RCLCPP_INFO(rclcpp::get_logger("base_navigator"),
                      "Got new goal pose");
          destination.SetData({msg.pose.position.x, msg.pose.position.y});
        });

    params.planningResolution =
        node.get_parameter_or("planning_resolution", 0.1);
    params.vehicleRadius = node.get_parameter_or("vehicle_radius", 0.3);
    params.maxPlanningTime = std::chrono::duration<double>(
        node.get_parameter_or("max_planning_time", 0.5));
  }
  bool HasNewCommand() override {
    const bool tmp = destination.HasNewData();
    destination.ResetFlag();
    if (tmp || path.size() < 2) {
      return tmp;
    }
    const size_t len = path.size();
    for (size_t i = 1; i < len; ++i) {
      if (!map->GetMap().IsSafePath(path[i - 1], path[i])) {
        return true;
      }
    }
    return false;
  };

 protected:
  std::vector<owen_common::types::Point2D> path;
  owen_common::types::MailboxData<owen_common::types::Point2D> destination;
  struct Parameters {
    float planningResolution;
    float vehicleRadius;
    std::chrono::duration<double> maxPlanningTime;
  };

  Parameters params;

 private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      destinationSub;
};

}  // namespace Navigation::PathGenerators
