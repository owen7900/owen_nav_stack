#pragma once

#include <string>

#include "owen_common/Pose2D.hpp"
#include "owen_navigation/mapping/Map.hpp"
namespace Navigation::Mapping::ObstacleSources {

class BaseObstacleSource {
 public:
  explicit BaseObstacleSource(const std::string& n) : name(n){};
  virtual ~BaseObstacleSource() = default;
  virtual Map::MapUpdate GetMapUpdate(
      const owen_common::types::Pose2D& pose) = 0;
  virtual bool HasMapUpdate() const = 0;
  std::string GetName() const { return name; }

 private:
  std::string name;
};
}  // namespace Navigation::Mapping::ObstacleSources
