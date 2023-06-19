#pragma once

#include "owen_navigation/path_followers/BasePathFollower.hpp"
namespace Navigation::PathFollowers::PathFollowerFactory {
std::unique_ptr<BasePathFollower> GetFollower(
    rclcpp::Node& n, const std::shared_ptr<Mapping::MapManager>& map,
    const std::string& name);
}  // namespace Navigation::PathFollowers::PathFollowerFactory
