#include "owen_navigation/NavigatorNode.hpp"

#include <random>
#include <rclcpp/logger.hpp>

#include "owen_navigation/path_followers/PathFollowerFactory.hpp"
#include "owen_navigation/path_generators/PathGeneratorFactory.hpp"
#include "owen_navigation/recovery_behaviours/RecoveryBehaviourFactory.hpp"

namespace Navigation {

namespace Constants {
constexpr double DefaultPoseTimeout = 0.1;
constexpr double DefaultMaxRecoveryTime = 5.0;
}  // namespace Constants

NavigatorNode::NavigatorNode(const std::string& name)
    : rclcpp::Node(name), recoveryBehaviourStartTime(0.0) {
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

  const auto recoveryBehaviourStrings = this->get_parameter_or(
      "recovery_behaviours", std::vector<std::string>{"back_up"});
  for (const auto& behaviourString : recoveryBehaviourStrings) {
    auto rec = RecoveryBehaviourFactory::GetRecoveryBehaviour(behaviourString,
                                                              *this, map);
    if (rec) {
      this->recoveryBehaviours.push_back(rec);
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
  maxRecoveryTime = this->get_parameter_or("max_recovery_time",
                                           Constants::DefaultMaxRecoveryTime);
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

void NavigatorNode::executeRecovery() {
  if (recoveryBehaviours.empty()) {
    this->commandPub->publish({});
  }

  const double now = this->get_clock()->now().seconds();

  if (now - this->recoveryBehaviourStartTime > this->maxRecoveryTime ||
      !activeRecoveryBehaviour ||
      !activeRecoveryBehaviour->IsRecoveryCommandActive()) {
    this->recoveryBehaviourStartTime = now;
    activeRecoveryBehaviour = nullptr;

    while (!activeRecoveryBehaviour) {
      const size_t randomRec = std::rand() % recoveryBehaviours.size();

      if (this->recoveryBehaviours[randomRec]->IsRecoveryCommandActive()) {
        activeRecoveryBehaviour = this->recoveryBehaviours[randomRec];
      }
    }
  }

  this->commandPub->publish(activeRecoveryBehaviour->GetRecoveryCommand());
}

}  // namespace Navigation
