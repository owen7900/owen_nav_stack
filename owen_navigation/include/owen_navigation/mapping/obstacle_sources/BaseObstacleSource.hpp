#pragma once

#include "owen_navigation/mapping/Map.hpp"
namespace Navigation::Mapping::ObstacleSources {

class BaseObstacleSource {
 public:
  virtual ~BaseObstacleSource() = default;
  virtual Map::MapUpdate GetMapUpdate() const = 0;
};
}  // namespace Navigation::Mapping::ObstacleSources
