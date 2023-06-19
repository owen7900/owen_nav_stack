#pragma once

#include "owen_common/Pose2D.hpp"
#include "owen_navigation/mapping/Map.hpp"
namespace Navigation::Mapping::ObstacleSources {

class BaseObstacleSource {
 public:
  virtual ~BaseObstacleSource() = default;
  virtual Map::MapUpdate GetMapUpdate(
      const owen_common::types::Pose2D& pose) const = 0;
};
}  // namespace Navigation::Mapping::ObstacleSources
