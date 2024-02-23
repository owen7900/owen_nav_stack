#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <utility>
#include <vector>

#include "owen_common/MailboxData.hpp"
#include "owen_common/Point2D.hpp"
#include "owen_navigation/mapping/MapManager.hpp"

namespace Navigation::PathFollowers {
class BasePathFollower {
 public:
  using Point = owen_common::types::Point2D;
  using Path = std::vector<Point>;
  using Command = geometry_msgs::msg::Twist;

 public:
  explicit BasePathFollower(std::shared_ptr<Mapping::MapManager> map_)
      : map(std::move(map_)){};
  virtual ~BasePathFollower() = default;

  void UpdatePath(const Path& p) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("base_path_follower"),
                       "Got path of size: " << p.size());
    path.SetData(p);
  }
  bool IsArrived() const { return isArrived; }

  virtual std::optional<Command> CalculateCommand(
      const owen_common::types::Pose2D& pose) = 0;

 protected:
  std::shared_ptr<Mapping::MapManager> map;
  owen_common::types::MailboxData<Path> path;
  bool isArrived;
};
}  // namespace Navigation::PathFollowers
