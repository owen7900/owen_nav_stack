#include "owen_navigation/mapping/Map.hpp"

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
  Point2D maxPt{resolution * width, resolution * height};
  Point2D newMax{std::max(pt.x, maxPt.x), std::max(pt.y, maxPt.y)};
  size_t newWidth = newMax.x / resolution;
  size_t newHeight = newMax.y / resolution;

  Point2D newOrigin;
  newOrigin.x = std::min(origin.x, pt.x);
  newOrigin.y = std::min(origin.y, pt.y);

  std::vector<bool> newMap(newWidth * newHeight, false);
  if (!map.empty()) {
    const size_t heightOffset = height - newHeight;
    const size_t widthOffset = width - newWidth;
    size_t idx = 0;
    for (size_t y = 0; y < newHeight; ++y) {
      for (size_t x = 0; x < newWidth; ++x) {
        size_t xOffset = x - widthOffset;
        size_t yOffset = y - heightOffset;
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
  return (pt.x > origin.x) && (pt.y > origin.y) && (GetIdx(pt) < map.size());
}

bool Map::IsInBounds(const IntPoint& pt) const {
  return pt.x < width && pt.y < height;
}

bool Map::IsInBounds(size_t idx) const { return idx < map.size(); }

size_t Map::GetIdx(const Point2D& pt) const {
  return GetIdx(GetCellCoords(pt));
}

size_t Map::GetIdx(const IntPoint& pt) const { return pt.x + width * pt.y; }

Map::IntPoint Map::GetCellCoords(const Point2D& pt) const {
  return {static_cast<size_t>(pt.x / width),
          static_cast<size_t>(pt.y / height)};
}

Map::IntPoint Map::GetCellCoords(size_t idx) const {
  return {idx % width, idx / width};
}

Map::Point2D Map::GetPoint(const IntPoint& pt) const {
  return origin + Point2D{pt.x * resolution, pt.y * resolution};
}

Map::Point2D Map::GetPoint(size_t idx) const {
  return GetPoint(GetCellCoords(idx));
}

void Map::UpdateMap(const Cell& update) {
  auto minIntPt = GetCellCoords(update.bounds.GetMinPt());
  auto maxIntPt = GetCellCoords(update.bounds.GetMaxPt());

  for (size_t y = minIntPt.y; y <= maxIntPt.y; ++y) {
    for (size_t x = minIntPt.x; x <= maxIntPt.x; ++x) {
      const size_t idx = GetIdx(IntPoint{x, y});
      if (IsInBounds(idx)) {
        map[idx] = update.state;
      }
    }
  }
}

void Map::UpdateMap(const MapUpdate& update) {
  Point2D minPt = origin;
  Point2D maxPt{width * resolution, height * resolution};

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

}  // namespace Navigation::Mapping
