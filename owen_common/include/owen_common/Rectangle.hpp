#pragma once

#include <algorithm>

#include "owen_common/Point2D.hpp"

namespace owen_common::types {
template <typename T>
class Rectangle {
 public:
  Rectangle() = default;
  Rectangle(const BasePoint2D<T>& pt1, const BasePoint2D<T>& pt2) {
    minPt.x = std::min(pt1.x, pt2.x);
    minPt.y = std::min(pt1.y, pt2.y);

    maxPt.x = std::max(pt1.x, pt2.x);
    maxPt.y = std::max(pt1.y, pt2.y);
  }

  BasePoint2D<T> GetMaxPt() const { return maxPt; };
  BasePoint2D<T> GetMinPt() const { return minPt; };

  friend std::ostream& operator<<(std::ostream& stream, const Rectangle& r) {
    stream << "{" << r.minPt << ", " << r.maxPt << "}";
    return stream;
  }

 private:
  BasePoint2D<T> minPt;
  BasePoint2D<T> maxPt;
};

}  // namespace owen_common::types
