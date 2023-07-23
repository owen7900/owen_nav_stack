#include "owen_navigation/mapping/Map.hpp"

#include <iostream>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace Navigation::Mapping {

namespace Constants {
constexpr double DefaultSize = 10.0;
constexpr double DefaultResolution = 0.1;
}  // namespace Constants

Map::Map()
    : Map({0.0, 0.0}, {Constants::DefaultSize, Constants::DefaultSize},
          Constants::DefaultResolution) {}

Map::Map(const Point2D& minPt, const Point2D& maxPt, double res)
    : width(0), height(0), origin(minPt), resolution(res) {
  origin.x = std::min(origin.x, minPt.x);
  origin.y = std::min(origin.y, minPt.y);
  Point2D newMax{std::max(minPt.x, maxPt.x), std::max(minPt.y, maxPt.y)};
  this->ResizeToPoint(newMax);
}

void Map::ResizeToPoint(const Point2D& pt) {
  if (IsInBounds(pt)) {
    return;
  }
  Point2D maxPt = origin + Point2D{resolution * width, resolution * height};
  Point2D newMax{std::max(pt.x, maxPt.x), std::max(pt.y, maxPt.y)};

  Point2D newOrigin;
  newOrigin.x = std::min(origin.x, pt.x);
  newOrigin.y = std::min(origin.y, pt.y);

  int newWidth = (newMax.x - newOrigin.x) / resolution;
  int newHeight = (newMax.y - newOrigin.y) / resolution;

  std::vector<bool> newMap(newWidth * newHeight, false);
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
}

bool Map::IsInBounds(const Point2D& pt) const {
  return IsInBounds(GetCellCoords(pt));
}

bool Map::IsInBounds(const IntPoint& pt) const {
  return pt.x < width && pt.y < height && pt.x >= 0 && pt.y >= 0;
}

bool Map::IsInBounds(int idx) const { return idx < map.size() && idx >= 0; }

int Map::GetIdx(const Point2D& pt) const { return GetIdx(GetCellCoords(pt)); }

int Map::GetIdx(const IntPoint& pt) const { return pt.x + width * pt.y; }

Map::Point2D Map::GetMaxPoint() const {
  return origin + Point2D{width * resolution, height * resolution};
}

Map::IntPoint Map::GetCellCoords(const Point2D& pt) const {
  const auto tmpPt = (pt - origin) / resolution;
  return {static_cast<int>(tmpPt.x), static_cast<int>(tmpPt.y)};
}

Map::IntPoint Map::GetCellCoords(int idx) const {
  return {idx % width, idx / width};
}

Map::Point2D Map::GetPoint(const IntPoint& pt) const {
  return origin + Point2D{pt.x * resolution, pt.y * resolution};
}

Map::Point2D Map::GetPoint(int idx) const {
  return GetPoint(GetCellCoords(idx));
}

void Map::UpdateMap(const Cell& update) {
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
}

void Map::UpdateMap(const MapUpdate& update) {
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
}

double Map::GetClosestObstacleDistance(const Point2D& pt,
                                       double searchRadius) const {
  const int searchRadiusX = std::min<int>(width / 2, searchRadius / resolution);
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
}

bool Map::IsSafe(const Point2D& pt, double vehicleRadius) const {
  const auto searchRadius =
      static_cast<int>(std::abs(vehicleRadius / resolution)) + 1;
  const auto startPt = GetCellCoords(pt);

  for (int y = startPt.y - searchRadius; y < startPt.y + searchRadius; ++y) {
    for (int x = startPt.x - searchRadius; x < startPt.x + searchRadius; ++x) {
      if (IsOccupied(IntPoint{x, y})) {
        return false;
      }
    }
  }

  return true;
}

bool Map::IsSafePath(const Point2D& pt, const Point2D& pt2,
                     double vehicleRadius) const {
  const double obsRadius = (vehicleRadius != 0 ? vehicleRadius : resolution);

  const int count = std::abs(pt.distanceFromPoint(pt2) / obsRadius) + 1;
  const auto step = (pt - pt2) / count;

  for (int i = 0; i < count; ++i) {
    if (!IsSafe(pt + (step * i), vehicleRadius)) {
      return false;
    }
  }

  return true;
}

}  // namespace Navigation::Mapping
