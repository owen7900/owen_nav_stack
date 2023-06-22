#pragma once
#include <rclcpp/node.hpp>

#include "owen_navigation/mapping/MapManager.hpp"
#include "owen_navigation/recovery_behaviours/BaseRecoveryBehaviour.hpp"

namespace Navigation::RecoveryBehaviourFactory {
std::shared_ptr<Navigation::RecoveryBehaviours::BaseRecoveryBehaviour>
GetRecoveryBehaviour(const std::string& name, rclcpp::Node& n,
                     const std::shared_ptr<Mapping::MapManager>& map);
}  // namespace Navigation::RecoveryBehaviourFactory
