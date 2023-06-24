#include "owen_navigation/recovery_behaviours/BackUpBehaviour.hpp"

namespace Navigation::RecoveryBehaviours {

namespace Constants {
const std::string Name = "back_up";
const std::string BackUpSpeedName = Name + "_speed";
constexpr double DefaultBackUpSpeed = 0.1;
const std::vector<std::string> AllParams{BackUpSpeedName};
}  // namespace Constants

BackUp::BackUp(rclcpp::Node& n) : params{} {
  this->setDefaultParamValues(n);
  paramsCallbackHandle = n.add_on_set_parameters_callback(
      [this](const auto& params) { return this->updateParams(params); });
  this->updateParams(n.get_parameters(Constants::AllParams));
}

rcl_interfaces::msg::SetParametersResult BackUp::updateParams(
    const std::vector<rclcpp::Parameter>& paramVec) {
  rcl_interfaces::msg::SetParametersResult ret;
  ret.successful = true;
  for (const auto& param : paramVec) {
    if (param.get_name() == Constants::BackUpSpeedName) {
      params.backUpSpeed = param.as_double();
    }
  }

  return ret;
}

void BackUp::setDefaultParamValues(rclcpp::Node& n) {
  n.declare_parameter(Constants::BackUpSpeedName,
                      Constants::DefaultBackUpSpeed);
  params.backUpSpeed = Constants::DefaultBackUpSpeed;
}

BaseRecoveryBehaviour::DriveCommand BackUp::GetRecoveryCommand() {
  BaseRecoveryBehaviour::DriveCommand cmd;
  cmd.linear.x = -params.backUpSpeed;
  return cmd;
}

}  // namespace Navigation::RecoveryBehaviours
