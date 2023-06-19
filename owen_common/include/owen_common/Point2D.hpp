#pragma once

#include <cmath>
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

  BasePoint2D operator-() { return {-x, -y}; }

  friend BasePoint2D operator-(const BasePoint2D& rhs, const BasePoint2D& lhs) {
    return rhs + -lhs;
  }

  double distanceFromPoint(const BasePoint2D& other) const {
    return std::sqrt(std::pow(other.x - x, 2) + std::pow(other.y - y, 2));
  }
};

using Point2D = BasePoint2D<double>;

}  // namespace owen_common::types
