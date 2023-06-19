#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "owen_navigation/mapping/MapManager.hpp"
#include "owen_navigation/path_generators/BasePathGenerator.hpp"

namespace Navigation {

class NavigatorNode : public rclcpp::Node {
 public:
  explicit NavigatorNode(const std::string& name);

 private:
  void controlLoop();

 private:
  std::vector<std::shared_ptr<PathGenerators::BasePathGenerator>>
      pathGenerators;

  std::shared_ptr<PathGenerators::BasePathGenerator> activeGenerator;

  std::vector<owen_common::types::Point2D> path;

  std::shared_ptr<Mapping::MapManager> map;

  rclcpp::TimerBase::SharedPtr controlLoopTimer;
};

}  // namespace Navigation
