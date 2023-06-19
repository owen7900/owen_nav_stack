#pragma once

#include <rclcpp/node.hpp>
#include <string>

#include "owen_navigation/mapping/obstacle_sources/BaseObstacleSource.hpp"
namespace Navigation::Mapping::ObstacleSourceFactory {
std::unique_ptr<ObstacleSources::BaseObstacleSource> GetObstacleSource(
    const std::string& name, rclcpp::Node& node);
}  // namespace Navigation::Mapping::ObstacleSourceFactory
