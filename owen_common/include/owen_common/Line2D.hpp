#pragma once

#include <owen_common/Point2D.hpp>

namespace owen_common::types {

template <typename T>
struct BaseLine2D {
  using PointT = BasePoint2D<T>;
  BaseLine2D(const PointT& p1_, const PointT& p2_)
      : p1(p1_), p2(p2_), angle(std::atan2(p2.y - p1.y, p2.x - p1.x)) {}

  PointT ProjectFromP1(const T& distance) const {
    return p1 + PointT{std::cos(angle) * distance, std::sin(angle) * distance};
  }

  [[nodiscard]] double GetAngle() const { return angle; }
  [[nodiscard]] double GetLength() const { return p1.distanceFromPoint(p2); }

 private:
  PointT p1;
  PointT p2;
  double angle;
};

}  // namespace owen_common::types
