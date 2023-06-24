#pragma once

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>

#include "owen_navigation/recovery_behaviours/BaseRecoveryBehaviour.hpp"

namespace Navigation::RecoveryBehaviours {

class BackUp : public BaseRecoveryBehaviour {
 public:
  explicit BackUp(rclcpp::Node& n);

  BaseRecoveryBehaviour::DriveCommand GetRecoveryCommand() override;

  bool IsRecoveryCommandActive() const override { return true; }

 private:
  rcl_interfaces::msg::SetParametersResult updateParams(
      const std::vector<rclcpp::Parameter>& paramVec);

  void setDefaultParamValues(rclcpp::Node& n);

 private:
  struct Params {
    double backUpSpeed;
  };
  Params params;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      paramsCallbackHandle;
};

}  // namespace Navigation::RecoveryBehaviours
