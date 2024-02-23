#include "owen_navigation/path_generators/PathGeneratorFactory.hpp"

#include "owen_navigation/path_generators/AStarNavigator.hpp"
#include "owen_navigation/path_generators/CoveragePathGenerator.hpp"
#include "owen_navigation/path_generators/RRTNavigator.hpp"

namespace Navigation {

std::shared_ptr<PathGenerators::BasePathGenerator> GetPathGenerator(
    rclcpp::Node& node, const std::string& name,
    const std::shared_ptr<Mapping::MapManager>& map) {
  if (name == "astar") {
    return std::make_shared<Navigation::PathGenerators::AStarNavigator>(node,
                                                                        map);
  }
  if (name == "rrt") {
    return std::make_shared<Navigation::PathGenerators::RRTNavigator>(node,
                                                                      map);
  }
  if (name == "vacuum") {
    return std::make_shared<Navigation::PathGenerators::CoveragePathGenerator>(
        node, map);
  }

  return nullptr;
}

}  // namespace Navigation
