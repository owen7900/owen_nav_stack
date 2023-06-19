#pragma once

#include <rclcpp/rclcpp.hpp>
#include <utility>

#include "owen_common/Point2D.hpp"
#include "owen_common/Pose2D.hpp"
#include "owen_navigation/mapping/MapManager.hpp"

namespace Navigation::PathGenerators {

class BasePathGenerator {
 public:
  virtual ~BasePathGenerator() = default;
  BasePathGenerator(rclcpp::Node& /*node*/, std::string generator_name,
                    std::shared_ptr<Mapping::MapManager> map)
      : pose(), map(std::move(map)), name(std::move(generator_name)){};

  std::string GetName() const { return name; };

  virtual std::vector<owen_common::types::Point2D> GeneratePath(
      const owen_common::types::Pose2D& pose) = 0;

  // this function should reset back to false even if a path is not generated
  // for the new command
  virtual bool HasNewCommand() = 0;

  virtual bool HasUpdatedPath() const = 0;

  void UpdatePose(const owen_common::types::Pose2D& pose_) { pose = pose_; };

 protected:
  owen_common::types::Pose2D pose;

  std::shared_ptr<Mapping::MapManager> map;

 private:
  std::string name;
};

}  // namespace Navigation::PathGenerators
