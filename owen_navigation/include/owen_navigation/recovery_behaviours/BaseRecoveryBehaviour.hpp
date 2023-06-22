#pragma once

#include <geometry_msgs/msg/detail/twist__struct.hpp>
namespace Navigation::RecoveryBehaviours {

class BaseRecoveryBehaviour {
 public:
  virtual ~BaseRecoveryBehaviour() = default;
  using DriveCommand = geometry_msgs::msg::Twist;

 public:
  virtual DriveCommand GetRecoveryCommand() = 0;
  virtual bool IsRecoveryCommandActive() const = 0;
};

}  // namespace Navigation::RecoveryBehaviours
