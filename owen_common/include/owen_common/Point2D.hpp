#pragma once

namespace owen_common::types {

template <typename T>
struct BasePoint2D {
  T x;
  T y;

  template <typename U>
  bool operator<(const BasePoint2D<U>& pt) {
    return (x == pt.x) ? (y < pt.y) : (x < pt.x);
  };

  friend BasePoint2D operator+(const BasePoint2D& lhs, const BasePoint2D& rhs) {
    return BasePoint2D{lhs.x + rhs.x, lhs.y + rhs.y};
  }
};

using Point2D = BasePoint2D<double>;

}  // namespace owen_common::types
