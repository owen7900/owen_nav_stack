#pragma once

#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <utility>

#include "owen_common/Point2D.hpp"
#include "owen_common/Pose2D.hpp"
#include "owen_navigation/mapping/MapManager.hpp"

namespace Navigation::PathGenerators {

class BasePathGenerator {
 public:
  virtual ~BasePathGenerator() = default;
  BasePathGenerator(rclcpp::Node& node, std::string generator_name,
                    std::shared_ptr<Mapping::MapManager> map)
      : pose(), map(std::move(map)), name(std::move(generator_name)) {
    debugPathPub =
        node.create_publisher<nav_msgs::msg::Path>(name + "/path", 1);
  };

  [[nodiscard]] std::string GetName() const { return name; };

  virtual std::vector<owen_common::types::Point2D> GeneratePath(
      const owen_common::types::Pose2D& pose) = 0;

  // this function should reset back to false even if a path is not generated
  // for the new command
  virtual bool HasNewCommand() = 0;

  [[nodiscard]] virtual bool HasUpdatedPath() const = 0;

  void UpdatePose(const owen_common::types::Pose2D& pose_) { pose = pose_; };

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
  owen_common::types::Pose2D pose;

  std::shared_ptr<Mapping::MapManager> map;

 private:
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr debugPathPub;
  std::string name;
};

}  // namespace Navigation::PathGenerators
