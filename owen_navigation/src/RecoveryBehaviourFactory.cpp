#include "owen_navigation/recovery_behaviours/RecoveryBehaviourFactory.hpp"

#include "owen_navigation/recovery_behaviours/BackUpBehaviour.hpp"
namespace Navigation::RecoveryBehaviourFactory {
std::shared_ptr<Navigation::RecoveryBehaviours::BaseRecoveryBehaviour>
GetRecoveryBehaviour(const std::string& name, rclcpp::Node& n,
                     const std::shared_ptr<Mapping::MapManager>& /*map*/) {
  if (name == "back_up") {
    return std::make_shared<Navigation::RecoveryBehaviours::BackUp>(n);
  }
  return nullptr;
}
}  // namespace Navigation::RecoveryBehaviourFactory
