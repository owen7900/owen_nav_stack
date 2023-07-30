#pragma once
#include <cmath>
#include <vector>

#include "owen_common/Point2D.hpp"
#include "owen_common/Rectangle.hpp"
namespace Navigation::Mapping {

template <typename T>
class GridMap {
 public:
  using Point2D = owen_common::types::Point2D;
  using IntPoint = owen_common::types::BasePoint2D<int>;
  using Rectangle = owen_common::types::Rectangle<double>;

  struct Cell {
    Rectangle bounds{};
    T state{};
  };
  using MapUpdate = std::vector<Cell>;
  constexpr static T Occupied = std::numeric_limits<T>::max();
  constexpr static T Free = std::numeric_limits<T>::min();

 private:
  constexpr static const double DefaultSize = 10.0;
  constexpr static const Point2D ExtraResize = {DefaultSize, DefaultSize};

 public:
  GridMap() = default;
  GridMap(const Point2D& minPt, const Point2D& maxPt, double resolution)
      : width(0), height(0), origin(minPt), resolution(resolution) {
    origin.x = std::min(origin.x, minPt.x);
    origin.y = std::min(origin.y, minPt.y);
    Point2D newMax{std::max(minPt.x, maxPt.x), std::max(minPt.y, maxPt.y)};
    this->ResizeToPoint(newMax);
  };

  void ResizeToPoint(const Point2D& pt) {
    if (IsInBounds(pt)) {
      return;
    }
    Point2D maxPt = origin + Point2D{resolution * width, resolution * height};
    Point2D newMax{std::max(pt.x, maxPt.x), std::max(pt.y, maxPt.y)};
    newMax = newMax + ExtraResize;

    Point2D newOrigin;
    newOrigin.x = std::min(origin.x, pt.x);
    newOrigin.y = std::min(origin.y, pt.y);
    newOrigin = newOrigin - ExtraResize;

    int newWidth = (newMax.x - newOrigin.x) / resolution;
    int newHeight = (newMax.y - newOrigin.y) / resolution;

    std::vector<T> newMap(newWidth * newHeight, false);
    if (!map.empty()) {
      const int heightOffset = height - newHeight;
      const int widthOffset = width - newWidth;
      int idx = 0;
      for (int y = 0; y < newHeight; ++y) {
        for (int x = 0; x < newWidth; ++x) {
          int xOffset = x - widthOffset;
          int yOffset = y - heightOffset;
          IntPoint ipt{xOffset, yOffset};
          if (IsInBounds(ipt)) {
            newMap[idx] = map[GetIdx(ipt)];
          }
          ++idx;
        }
      }
    }

    map = std::move(newMap);
    origin = newOrigin;
    height = newHeight;
    width = newWidth;
  };

  inline bool IsOccupied(const Point2D& pt) const {
    return IsInBounds(pt) ? map.at(GetIdx(pt)) != Free : false;
  };
  inline bool IsOccupied(const IntPoint& pt) const {
    return IsInBounds(pt) ? map.at(GetIdx(pt)) != Free : false;
  };
  inline bool IsOccupied(size_t idx) const {
    return IsInBounds(idx) ? map.at(idx) != Free : false;
  };

  double GetClosestObstacleDistance(
      const Point2D& pt,
      double searchRadius = std::numeric_limits<double>::max()) const {
    const int searchRadiusX =
        std::min<int>(width / 2, searchRadius / resolution);
    const int searchRadiusY =
        std::min<int>(height / 2, searchRadius / resolution);
    double ret = std::numeric_limits<double>::max();
    const auto startPt = GetCellCoords(pt);

    for (int y = 0; y < searchRadiusY; ++y) {
      for (int x = 0; x < searchRadiusX; ++x) {
        auto checkPt = startPt + IntPoint{x, y};
        if (IsOccupied(checkPt)) {
          ret = std::min(ret, std::pow(x, 2) + std::pow(y, 2));
          continue;
        }
        checkPt = startPt + IntPoint{-x, -y};
        if (startPt.x >= x && startPt.y >= y && IsOccupied(checkPt)) {
          ret = std::min(ret, std::pow(x, 2) + std::pow(y, 2));
          continue;
        }
        checkPt = startPt + IntPoint{-x, y};
        if (startPt.x >= x && IsOccupied(checkPt)) {
          ret = std::min(ret, std::pow(x, 2) + std::pow(y, 2));
          continue;
        }
        checkPt = startPt + IntPoint{x, -y};
        if (startPt.y >= y && IsOccupied(checkPt)) {
          ret = std::min(ret, std::pow(x, 2) + std::pow(y, 2));
          continue;
        }
      }
    }

    return std::sqrt(ret) * resolution;
  };

  bool IsSafe(const Point2D& pt, double vehicleRadius = 0) const {
    const auto searchRadius =
        static_cast<int>(std::abs(vehicleRadius / resolution)) + 1;
    const auto startPt = GetCellCoords(pt);

    for (int y = startPt.y - searchRadius; y < startPt.y + searchRadius; ++y) {
      for (int x = startPt.x - searchRadius; x < startPt.x + searchRadius;
           ++x) {
        if (IsOccupied(IntPoint{x, y})) {
          return false;
        }
      }
    }

    return true;
  };
  bool IsSafePath(const Point2D& pt, const Point2D& pt2,
                  double vehicleRadius = 0) const {
    const int obsRadius = std::ceil(
        (vehicleRadius != 0 ? vehicleRadius : resolution) / resolution);

    const auto ipt1 = GetCellCoords(pt);
    const auto ipt2 = GetCellCoords(pt2);
    if (ipt1.y == ipt2.y) {
      const int minX = std::min(ipt1.x, ipt2.x) - obsRadius;
      const int maxX = std::max(ipt1.x, ipt2.x) + obsRadius;
      for (int y = ipt1.y - obsRadius; y < ipt1.y + obsRadius; ++y) {
        for (int x = minX; x <= maxX; ++x) {
          IntPoint ipt{x, y};
          if (IsOccupied(ipt)) {
            return false;
          }
        }
      }
      return true;
    }

    int minY = 0;
    int maxY = height;
    int minX = 0;
    int maxX = 0;
    double slope = 0;
    if (ipt1.y < ipt2.y) {
      minY = std::max(0, ipt1.y - obsRadius);
      maxY = std::min(height, ipt2.y + obsRadius);
      minX = ipt1.x - obsRadius;
      maxX = ipt1.x + obsRadius;
      slope = (static_cast<double>(ipt1.x) - static_cast<double>(ipt2.x)) /
              (static_cast<double>(ipt1.y) - static_cast<double>(ipt2.y));
    } else {
      minY = std::max(0, ipt2.y - obsRadius);
      maxY = std::min(height, ipt1.y + obsRadius);
      minX = ipt2.x - obsRadius;
      maxX = ipt2.x + obsRadius;
      slope = (static_cast<double>(ipt2.x) - static_cast<double>(ipt1.x)) /
              (static_cast<double>(ipt2.y) - static_cast<double>(ipt1.y));
    }

    for (int y = minY; y <= maxY;
         ++y, minX = std::floor(minX + slope), maxX = std::ceil(maxX + slope)) {
      for (int x = minX; x <= maxX; ++x) {
        const IntPoint ipt{x, y};
        if (IsOccupied(ipt)) {
          return false;
        }
      }
    }

    return true;
  };

  inline bool IsInBounds(const Point2D& pt) const {
    return IsInBounds(GetCellCoords(pt));
  };
  inline bool IsInBounds(const IntPoint& pt) const {
    return pt.x < width && pt.y < height && pt.x >= 0 && pt.y >= 0;
  };
  inline bool IsInBounds(int idx) const {
    return idx < map.size() && idx >= 0;
  };

  inline int GetIdx(const Point2D& pt) const {
    return GetIdx(GetCellCoords(pt));
  };
  inline int GetIdx(const IntPoint& pt) const { return pt.x + width * pt.y; };

  IntPoint GetCellCoords(const Point2D& pt) const {
    const auto tmpPt = (pt - origin) / resolution;
    return {static_cast<int>(tmpPt.x), static_cast<int>(tmpPt.y)};
  };
  inline IntPoint GetCellCoords(int idx) const {
    return {idx % width, idx / width};
  };

  inline Point2D GetPoint(const IntPoint& pt) const {
    return origin + Point2D{pt.x * resolution, pt.y * resolution};
  };
  inline Point2D GetPoint(int idx) const {
    return GetPoint(GetCellCoords(idx));
  };

  void UpdateMap(const MapUpdate& update) {
    Point2D minPt = origin;
    Point2D maxPt = Point2D{width * resolution, height * resolution} + origin;

    for (const auto& cell : update) {
      minPt.x = std::min(cell.bounds.GetMinPt().x, minPt.x);
      minPt.y = std::min(cell.bounds.GetMinPt().y, minPt.y);
      maxPt.x = std::max(cell.bounds.GetMaxPt().x, maxPt.x);
      maxPt.y = std::max(cell.bounds.GetMaxPt().y, maxPt.y);
    }

    if (!IsInBounds(minPt)) {
      this->ResizeToPoint(minPt);
    }
    if (!IsInBounds(maxPt)) {
      this->ResizeToPoint(maxPt);
    }

    for (const auto& cell : update) {
      UpdateMap(cell);
    }
  };

  void UpdateMap(const Cell& update) {
    this->ResizeToPoint(update.bounds.GetMinPt());
    this->ResizeToPoint(update.bounds.GetMaxPt());
    auto minIntPt = GetCellCoords(update.bounds.GetMinPt());
    auto maxIntPt = GetCellCoords(update.bounds.GetMaxPt());

    for (int y = minIntPt.y; y <= maxIntPt.y; ++y) {
      for (int x = minIntPt.x; x <= maxIntPt.x; ++x) {
        const auto pt = IntPoint{x, y};
        const int idx = GetIdx(pt);
        if (IsInBounds(idx)) {
          map[idx] = update.state;
        }
      }
    }
  };

  size_t GetWidth() const { return width; };
  size_t GetHeight() const { return height; };

  Point2D GetOrigin() const { return origin; };
  double GetResolution() const { return resolution; };

  const std::vector<T>& GetData() const { return map; };

  Point2D GetMaxPoint() const {
    return origin + Point2D{width * resolution, height * resolution};
  };

  void SetCell(int idx, T state) {
    if (IsInBounds(idx)) {
      map.at(idx) = state;
    }
  }

 private:
  std::vector<T> map;
  int width;
  int height;
  Point2D origin;
  double resolution;
};

}  // namespace Navigation::Mapping
