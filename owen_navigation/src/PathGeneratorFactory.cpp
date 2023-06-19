#include "owen_navigation/path_generators/PathGeneratorFactory.hpp"

#include "owen_navigation/path_generators/AStarNavigator.hpp"

namespace Navigation {

std::shared_ptr<PathGenerators::BasePathGenerator> GetPathGenerator(
    rclcpp::Node& node, const std::string& name) {
  if (name == "AStar") {
    return std::make_shared<Navigation::PathGenerators::AStarNavigator>(node);
  }

  return nullptr;
}

}  // namespace Navigation
