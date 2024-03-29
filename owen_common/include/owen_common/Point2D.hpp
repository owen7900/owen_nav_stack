#pragma once

#include <cmath>
#include <functional>
#include <ostream>
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

  friend BasePoint2D operator-(const BasePoint2D& lhs, const BasePoint2D& rhs) {
    return {lhs.x - rhs.x, lhs.y - rhs.y};
  }

  template <typename U>
  friend BasePoint2D operator/(const BasePoint2D& lhs, const U& rhs) {
    return {lhs.x / rhs, lhs.y / rhs};
  }

  template <typename U>
  friend BasePoint2D operator*(const BasePoint2D& lhs, const U& rhs) {
    return {lhs.x * rhs, lhs.y * rhs};
  }

  template <typename U>
  friend BasePoint2D operator*(const U& lhs, const BasePoint2D& rhs) {
    return {lhs * rhs.x, lhs * rhs.y};
  }

  double distanceFromPoint(const BasePoint2D& other) const {
    return std::sqrt(std::pow(other.x - x, 2) + std::pow(other.y - y, 2));
  }

  template <typename U>
  bool operator==(const BasePoint2D<U>& p) const {
    return p.x == x && p.y == y;
  }

  template <typename U>
  bool operator!=(const BasePoint2D<U>& p) const {
    return !(p == *this);
  }
};

struct BasePointHash {
  template <typename T>
  size_t operator()(const BasePoint2D<T>& pt) const {
    return std::hash<T>{}(pt.x) ^ std::hash<T>{}(pt.y);
  }
};

template <typename T>
std::ostream& operator<<(std::ostream& stream,
                         const owen_common::types::BasePoint2D<T>& pt) {
  stream << "{" << pt.x << ", " << pt.y << "}";
  return stream;
}

using Point2D = BasePoint2D<double>;

}  // namespace owen_common::types
