#pragma once

namespace owen_common::types {

template <typename T>
struct BasePose2D {
  T x;
  T y;
  T yaw;
};

using Pose2D = BasePose2D<double>;

}  // namespace owen_common::types
