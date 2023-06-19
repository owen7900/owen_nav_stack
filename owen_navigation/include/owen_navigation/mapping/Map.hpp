#pragma once
#include <cmath>
#include <vector>

#include "owen_common/Point2D.hpp"
#include "owen_common/Rectangle.hpp"
namespace Navigation::Mapping {

class Map {
 private:
  using Point2D = owen_common::types::Point2D;
  using IntPoint = owen_common::types::BasePoint2D<size_t>;
  using Rectangle = owen_common::types::Rectangle<double>;

 public:
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

  bool IsInBounds(const Point2D& pt) const;
  bool IsInBounds(const IntPoint& pt) const;
  bool IsInBounds(size_t idx) const;

  size_t GetIdx(const Point2D& pt) const;
  size_t GetIdx(const IntPoint& pt) const;

  IntPoint GetCellCoords(const Point2D& pt) const;
  IntPoint GetCellCoords(size_t idx) const;

  Point2D GetPoint(const IntPoint& pt) const;
  Point2D GetPoint(size_t idx) const;

  void UpdateMap(const MapUpdate& update);
  void UpdateMap(const Cell& update);

  size_t GetWidth() const { return width; };
  size_t GetHeight() const { return height; };

  Point2D GetOrigin() const { return origin; };
  double GetResolution() const { return resolution; };

  const std::vector<bool>& GetData() const { return map; };

 private:
  std::vector<bool> map;
  size_t width;
  size_t height;
  Point2D origin;
  double resolution;
};

}  // namespace Navigation::Mapping
