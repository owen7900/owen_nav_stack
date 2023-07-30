#pragma once

#include <string>
#include <utility>

#include "owen_common/Pose2D.hpp"
#include "owen_navigation/mapping/MapManager.hpp"
namespace Navigation::Mapping::ObstacleSources {

class BaseObstacleSource {
 public:
  explicit BaseObstacleSource(std::string n) : name(std::move(n)){};
  virtual ~BaseObstacleSource() = default;
  virtual MapManager::MapT::MapUpdate GetMapUpdate(
      const owen_common::types::Pose2D& pose) = 0;
  virtual bool HasMapUpdate() const = 0;
  std::string GetName() const { return name; }

 private:
  std::string name;
};
}  // namespace Navigation::Mapping::ObstacleSources
