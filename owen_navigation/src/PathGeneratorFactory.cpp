#include "owen_navigation/path_generators/PathGeneratorFactory.hpp"

#include "owen_navigation/path_generators/AStarNavigator.hpp"

namespace Navigation {

std::shared_ptr<PathGenerators::BasePathGenerator> GetPathGenerator(
    rclcpp::Node& node, const std::string& name,
    const std::shared_ptr<Mapping::MapManager>& map) {
  if (name == "AStar") {
    return std::make_shared<Navigation::PathGenerators::AStarNavigator>(node,
                                                                        map);
  }

  return nullptr;
}

}  // namespace Navigation
