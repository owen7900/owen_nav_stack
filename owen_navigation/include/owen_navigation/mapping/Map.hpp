#pragma once
#include <cmath>
#include <vector>

#include "owen_common/Point2D.hpp"
#include "owen_common/Rectangle.hpp"
namespace Navigation::Mapping {

class Map {
 public:
  using Point2D = owen_common::types::Point2D;
  using IntPoint = owen_common::types::BasePoint2D<int>;
  using Rectangle = owen_common::types::Rectangle<double>;

  struct Cell {
    Rectangle bounds{};
    bool state{};
  };
  using MapUpdate = std::vector<Cell>;

 public:
  Map();
  Map(const Point2D& minPt, const Point2D& maxPt, double resolution);

  void ResizeToPoint(const Point2D& pt);

  bool IsOccupied(const Point2D& pt) const {
    return IsInBounds(pt) ? map.at(GetIdx(pt)) : false;
  };
  bool IsOccupied(const IntPoint& pt) const {
    return IsInBounds(pt) ? map.at(GetIdx(pt)) : false;
  };
  bool IsOccupied(size_t idx) const {
    return IsInBounds(idx) ? map.at(idx) : false;
  };

  double GetClosestObstacleDistance(
      const Point2D& pt,
      double searchRadius = std::numeric_limits<double>::max()) const;

  bool IsSafe(const Point2D& pt, double vehicleRadius = 0) const;
  bool IsSafePath(const Point2D& pt, const Point2D& pt2,
                  double vehicleRadius = 0) const;

  bool IsInBounds(const Point2D& pt) const;
  bool IsInBounds(const IntPoint& pt) const;
  bool IsInBounds(int idx) const;

  int GetIdx(const Point2D& pt) const;
  int GetIdx(const IntPoint& pt) const;

  IntPoint GetCellCoords(const Point2D& pt) const;
  IntPoint GetCellCoords(int idx) const;

  Point2D GetPoint(const IntPoint& pt) const;
  Point2D GetPoint(int idx) const;

  void UpdateMap(const MapUpdate& update);
  void UpdateMap(const Cell& update);

  size_t GetWidth() const { return width; };
  size_t GetHeight() const { return height; };

  Point2D GetOrigin() const { return origin; };
  double GetResolution() const { return resolution; };

  const std::vector<bool>& GetData() const { return map; };

  Point2D GetMaxPoint() const;

  void SetCell(int idx, bool state) {
    if (IsInBounds(idx)) {
      map.at(idx) = state;
    }
  }

 private:
  std::vector<bool> map;
  int width;
  int height;
  Point2D origin;
  double resolution;
};

}  // namespace Navigation::Mapping
