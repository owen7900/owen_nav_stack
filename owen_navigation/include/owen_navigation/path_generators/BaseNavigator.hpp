
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
          destination.SetData({msg.pose.position.x, msg.pose.position.y});
        });
    debugPathPub = node.create_publisher<nav_msgs::msg::Path>("/path", 1);

    params.planningResolution =
        node.get_parameter_or("planning_resolution", 0.1);
    params.vehicleRadius = node.get_parameter_or("vehicle_radius", 0.15);
  }
  bool HasNewCommand() override {
    const bool tmp = destination.HasNewData();
    destination.ResetFlag();
    return tmp;
  };

 protected:
  void publishDebugPath(
      const std::vector<owen_common::types::Point2D>& path) const {
    nav_msgs::msg::Path p;
    p.header.frame_id = "map";
    p.poses.resize(path.size());
    std::transform(path.begin(), path.end(), p.poses.begin(),
                   [](const auto& pt) {
                     geometry_msgs::msg::PoseStamped r;
                     r.header.frame_id = "map";
                     r.pose.position.x = pt.x;
                     r.pose.position.y = pt.y;
                     return r;
                   });
    this->debugPathPub->publish(p);
  }

 protected:
  std::vector<owen_common::types::Point2D> path;
  owen_common::types::MailboxData<owen_common::types::Point2D> destination;
  struct Parameters {
    float planningResolution;
    float vehicleRadius;
  };

  Parameters params;

 private:
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr debugPathPub;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      destinationSub;
};

}  // namespace Navigation::PathGenerators
