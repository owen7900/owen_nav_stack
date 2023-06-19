#include "owen_navigation/NavigatorNode.hpp"

#include "owen_navigation/path_generators/PathGeneratorFactory.hpp"

namespace Navigation {

NavigatorNode::NavigatorNode(const std::string& name) : rclcpp::Node(name) {
  this->controlLoopTimer = this->create_wall_timer(
      std::chrono::duration<double>(0.05), [this] { controlLoop(); });

  const auto pathGeneratorsStrings = this->get_parameter_or(
      "path_generators", std::vector<std::string>{"AStar"});

  for (const auto& generatorString : pathGeneratorsStrings) {
    auto gen = GetPathGenerator(*this, generatorString);
    if (gen) {
      this->pathGenerators.push_back(std::move(gen));
    }
  }
}

void NavigatorNode::controlLoop() {
  for (const auto& gen : pathGenerators) {
    if (gen->HasNewCommand()) {
      this->activeGenerator = gen;
      this->path = gen->GeneratePath();
    }
  }

  if (this->activeGenerator->HasUpdatedPath()) {
    this->path = this->activeGenerator->GeneratePath();
  }
}

}  // namespace Navigation
