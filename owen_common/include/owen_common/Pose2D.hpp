#pragma once

#include <ostream>
namespace owen_common::types {

template <typename T>
struct BasePose2D {
  T x;
  T y;
  T yaw;
};

template <typename T>
std::ostream& operator<<(std::ostream& stream,
                         const owen_common::types::BasePose2D<T>& pt) {
  stream << "{{" << pt.x << ", " << pt.y << "}, yaw: " << pt.yaw << "}";
  return stream;
}

using Pose2D = BasePose2D<double>;

}  // namespace owen_common::types
