#pragma once
#include <cmath>
#include <iostream>
#include <vector>

#include "owen_common/Line2D.hpp"
#include "owen_common/Point2D.hpp"
#include "owen_common/Rectangle.hpp"
namespace Navigation::Mapping {

template <typename T, T Free = std::numeric_limits<T>::min(),
          T Occupied = std::numeric_limits<T>::max(),
          T Unknown = std::numeric_limits<T>::min()>
class GridMap {
 public:
  using Point2D = owen_common::types::Point2D;
  using IntPoint = owen_common::types::BasePoint2D<int>;
  using Rectangle = owen_common::types::Rectangle<double>;
  using CellVal = T;
  const static T FreeVal = Free;
  const static T OccupiedVal = Occupied;
  const static T UnknownVal = Unknown;

  struct Cell {
    Rectangle bounds{};
    T state{};
  };
  using MapUpdate = std::vector<Cell>;

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
    Clear();
  };

  void Clear() { Fill(Unknown); }

  void Fill(const T& val) { std::fill(map.begin(), map.end(), val); }

  void ResizeToPoint(const Point2D& pt) {
    if (IsInBounds(pt)) {
      return;
    }
    Point2D maxPt = origin + Point2D{resolution * width, resolution * height};
    Point2D newMax{std::max(pt.x, maxPt.x), std::max(pt.y, maxPt.y)};
    if (newMax != maxPt) {
      newMax = newMax + ExtraResize;
    }

    Point2D newOrigin;
    newOrigin.x = std::min(origin.x, pt.x);
    newOrigin.y = std::min(origin.y, pt.y);
    if (newOrigin != origin) {
      newOrigin = newOrigin - ExtraResize;
    }

    int newWidth = (newMax.x - newOrigin.x) / resolution;
    int newHeight = (newMax.y - newOrigin.y) / resolution;

    std::vector<T> newMap(newWidth * newHeight, Unknown);
    if (!map.empty()) {
      const int widthOffset = (origin.x - newOrigin.x) / resolution;
      const int heightOffset = (origin.y - newOrigin.y) / resolution;
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

  [[nodiscard]] inline bool IsOccupied(const Point2D& pt) const {
    if (!IsInBounds(pt)) {
      return false;
    }
    const T d = map.at(GetIdx(pt));
    return d != Free && d != Unknown;
  };
  [[nodiscard]] inline bool IsOccupied(const IntPoint& pt) const {
    if (!IsInBounds(pt)) {
      return false;
    }
    const T d = map.at(GetIdx(pt));
    return d != Free && d != Unknown;
  };
  [[nodiscard]] inline bool IsOccupied(size_t idx) const {
    if (!IsInBounds(idx)) {
      return false;
    }
    const T d = map.at(idx);
    return d != Free && d != Unknown;
  };

  [[nodiscard]] inline bool IsFree(const Point2D& pt) const {
    return IsInBounds(pt) ? map.at(GetIdx(pt)) == Free : false;
  };
  [[nodiscard]] inline bool IsFree(const IntPoint& pt) const {
    return IsInBounds(pt) ? map.at(GetIdx(pt)) == Free : false;
  };
  [[nodiscard]] inline bool IsFree(size_t idx) const {
    return IsInBounds(idx) ? map.at(idx) == Free : false;
  };

  [[nodiscard]] double GetClosestObstacleDistance(
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

  [[nodiscard]] bool IsSafe(const Point2D& pt, double vehicleRadius = 0) const {
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

  [[nodiscard]] bool IsSafe(const IntPoint& pt,
                            double vehicleRadius = 0) const {
    const auto searchRadius =
        static_cast<int>(std::abs(vehicleRadius / resolution)) + 1;

    for (int y = pt.y - searchRadius; y < pt.y + searchRadius; ++y) {
      for (int x = pt.x - searchRadius; x < pt.x + searchRadius; ++x) {
        if (IsOccupied(IntPoint{x, y})) {
          return false;
        }
      }
    }

    return true;
  }

  [[nodiscard]] bool IsSafePath(const Point2D& pt, const Point2D& pt2,
                                double vehicleRadius = 0) const {
    owen_common::types::BaseLine2D<double> line{pt, pt2};
    Point2D p = pt;
    double distance = 0;
    while (p.distanceFromPoint(pt2) > (resolution * 2)) {
      if (!IsSafe(p, vehicleRadius)) {
        return false;
      }
      distance += resolution;
      p = line.ProjectFromP1(distance);
    }
    return IsSafe(pt2, vehicleRadius);

    // const int obsRadius = std::ceil(
    //     (vehicleRadius != 0 ? vehicleRadius : resolution) / resolution);
    //
    // const auto ipt1 = GetCellCoords(pt);
    // const auto ipt2 = GetCellCoords(pt2);
    // if (ipt1.y == ipt2.y) {
    //   const int minX = std::min(ipt1.x, ipt2.x) - obsRadius;
    //   const int maxX = std::max(ipt1.x, ipt2.x) + obsRadius;
    //   for (int y = ipt1.y - obsRadius; y < ipt1.y + obsRadius; ++y) {
    //     for (int x = minX; x <= maxX; ++x) {
    //       IntPoint ipt{x, y};
    //       if (IsOccupied(ipt)) {
    //         return false;
    //       }
    //     }
    //   }
    //   return true;
    // }
    //
    // int minY = 0;
    // int maxY = 0;
    // int minX = 0;
    // int maxX = 0;
    // double slope = 0;
    // if (ipt1.y < ipt2.y) {
    //   minY = std::max(0, ipt1.y - obsRadius);
    //   maxY = std::min(height, ipt2.y + obsRadius);
    //   minX = ipt1.x - obsRadius;
    //   maxX = ipt1.x + obsRadius;
    //   slope = (static_cast<double>(ipt1.x) - static_cast<double>(ipt2.x)) /
    //           (static_cast<double>(ipt1.y) - static_cast<double>(ipt2.y));
    // } else {
    //   minY = std::max(0, ipt2.y - obsRadius);
    //   maxY = std::min(height, ipt1.y + obsRadius);
    //   minX = ipt2.x - obsRadius;
    //   maxX = ipt2.x + obsRadius;
    //   slope = (static_cast<double>(ipt2.x) - static_cast<double>(ipt1.x)) /
    //           (static_cast<double>(ipt2.y) - static_cast<double>(ipt1.y));
    // }
    //
    // for (int y = minY; y <= maxY;
    //      ++y, minX = std::floor(minX + slope), maxX = std::ceil(maxX +
    //      slope)) {
    //   for (int x = minX; x <= maxX; ++x) {
    //     const IntPoint ipt{x, y};
    //     if (IsOccupied(ipt)) {
    //       return false;
    //     }
    //   }
    // }
    //
    // return true;
  };

  [[nodiscard]] inline bool IsInBounds(const Point2D& pt) const {
    return IsInBounds(GetCellCoords(pt));
  };
  [[nodiscard]] inline bool IsInBounds(const IntPoint& pt) const {
    return pt.x < width && pt.y < height && pt.x >= 0 && pt.y >= 0;
  };
  [[nodiscard]] inline bool IsInBounds(int idx) const {
    return idx < map.size() && idx >= 0;
  };

  [[nodiscard]] inline int GetIdx(const Point2D& pt) const {
    return GetIdx(GetCellCoords(pt));
  };
  [[nodiscard]] inline int GetIdx(const IntPoint& pt) const {
    return pt.x + width * pt.y;
  };

  [[nodiscard]] IntPoint GetCellCoords(const Point2D& pt) const {
    const auto tmpPt = (pt - origin) / resolution;
    return {static_cast<int>(tmpPt.x), static_cast<int>(tmpPt.y)};
  };
  [[nodiscard]] inline IntPoint GetCellCoords(int idx) const {
    return {idx % width, idx / width};
  };

  [[nodiscard]] inline Point2D GetPoint(const IntPoint& pt) const {
    return origin + Point2D{pt.x * resolution, pt.y * resolution};
  };
  [[nodiscard]] inline Point2D GetPoint(int idx) const {
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
      std::cout << "resizing from update: " << minPt << "\n";
      this->ResizeToPoint(minPt);
    }
    if (!IsInBounds(maxPt)) {
      std::cout << "Resizeing fromupdaet: " << maxPt << "\n";
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

  [[nodiscard]] size_t GetWidth() const { return width; };
  [[nodiscard]] size_t GetHeight() const { return height; };

  [[nodiscard]] Point2D GetOrigin() const { return origin; };
  [[nodiscard]] double GetResolution() const { return resolution; };

  const std::vector<T>& GetData() const { return map; };

  [[nodiscard]] Point2D GetMaxPoint() const {
    return origin +
           Point2D{(width - 1) * resolution, (height - 1) * resolution};
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
