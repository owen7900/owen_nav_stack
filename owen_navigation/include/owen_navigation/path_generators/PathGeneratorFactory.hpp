#pragma once

#include <memory>

#include "owen_navigation/path_generators/BasePathGenerator.hpp"
namespace Navigation {

std::shared_ptr<PathGenerators::BasePathGenerator> GetPathGenerator(
    rclcpp::Node& node, const std::string& name);

}
