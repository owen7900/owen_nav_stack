#include "owen_navigation/NavigatorNode.hpp"

#include <rclcpp/logger.hpp>

#include "owen_navigation/path_followers/PathFollowerFactory.hpp"
#include "owen_navigation/path_generators/PathGeneratorFactory.hpp"

namespace Navigation {

namespace Constants {
constexpr double DefaultPoseTimeout = 0.1;
}

NavigatorNode::NavigatorNode(const std::string& name) : rclcpp::Node(name) {
  this->controlLoopTimer = this->create_wall_timer(
      std::chrono::duration<double>(0.05), [this] { controlLoop(); });
  map = std::make_shared<Mapping::MapManager>(*this);

  const auto pathGeneratorsStrings = this->get_parameter_or(
      "path_generators", std::vector<std::string>{"AStar"});

  for (const auto& generatorString : pathGeneratorsStrings) {
    auto gen = GetPathGenerator(*this, generatorString, map);
    if (gen) {
      this->pathGenerators.push_back(std::move(gen));
    }
  }

  this->pathFollower = PathFollowers::PathFollowerFactory::GetFollower(
      *this, map,
      this->get_parameter_or<std::string>("path_follower", "pure_pursuit"));

  if (!pathFollower) {
    RCLCPP_FATAL(this->get_logger(), "Path follower is nullptr");
    throw -1;
  }

  commandPub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

  dataTimeout =
      this->get_parameter_or("data_timeout", Constants::DefaultPoseTimeout);
}

void NavigatorNode::controlLoop() {
  this->map->UpdateMap(pose.GetDataRef());
  for (const auto& gen : pathGenerators) {
    if (gen->HasNewCommand()) {
      this->activeGenerator = gen;
      this->pathFollower->UpdatePath(gen->GeneratePath(pose.GetDataRef()));
    }
  }

  if (this->activeGenerator->HasUpdatedPath()) {
    this->pathFollower->UpdatePath(
        this->activeGenerator->GeneratePath(pose.GetDataRef()));
  }

  if (this->pose.GetDataAge() > dataTimeout) {
    this->commandPub->publish({});
  } else {
    auto command = this->pathFollower->CalculateCommand(pose.GetDataRef());
    if (command.has_value()) {
      this->commandPub->publish(command.value());
    } else {
      this->executeRecovery();
    }
  }
}

void NavigatorNode::executeRecovery() { this->commandPub->publish({}); }

}  // namespace Navigation
