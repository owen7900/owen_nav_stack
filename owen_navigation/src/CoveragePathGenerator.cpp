#include "owen_navigation/path_generators/CoveragePathGenerator.hpp"

#include <algorithm>
#include <iterator>
#include <utility>

#include "owen_common/AStar.hpp"

namespace Navigation::PathGenerators {
using namespace owen_common::types;
using IntPoint = Mapping::MapManager::MapT::IntPoint;
using AStar = owen_common::AStar<Mapping::MapManager::MapT>;

namespace Constants {
const std::string Name = "Coverage";
const double VehicleRadius = 0.15;
}  // namespace Constants

CoveragePathGenerator::CoveragePathGenerator(
    rclcpp::Node& n, std::shared_ptr<Mapping::MapManager> map)
    : BasePathGenerator(n, Constants::Name, std::move(map)),
      hasNewCommand_(false) {
  startCommandSub_ = n.create_subscription<std_msgs::msg::Empty>(
      "start_vacuum", 1, [this](std_msgs::msg::Empty::UniquePtr /*ptr*/) {
        hasNewCommand_ = true;
        coverageMap.clear();
      });
}

std::vector<owen_common::types::Point2D> CoveragePathGenerator::GeneratePath(
    const owen_common::types::Pose2D& pose) {
  initializeCoverageMap();
  const auto& m = map->GetMapRef();
  auto rbtCellCoords = m.GetCellCoords({pose.x, pose.y});
  std::vector<IntPoint> plan;
  std::optional<IntPoint> currPos{rbtCellCoords};
  while (currPos.has_value() && rclcpp::ok()) {
    plan.push_back(currPos.value());
    updateCoverageMap(currPos.value_or(plan.back()));
    currPos = getNextBestMove(currPos.value_or(plan.back()));
    while (currPos.has_value() && rclcpp::ok()) {
      plan.push_back(currPos.value_or(plan.back()));
      updateCoverageMap(currPos.value_or(plan.back()));
      currPos = getNextBestMove(currPos.value_or(plan.back()));
    }
    currPos = getNewStartingPoint(plan.back(), plan);
  }

  std::vector<owen_common::types::Point2D> ret(plan.size());
  std::transform(plan.begin(), plan.end(), ret.begin(),
                 [m](const auto& pt) { return m.GetPoint(pt); });
  publishDebugPath(ret);
  return ret;
}

bool CoveragePathGenerator::HasNewCommand() {
  const bool tmp = hasNewCommand_;
  hasNewCommand_ = false;
  return tmp;
}
void CoveragePathGenerator::initializeCoverageMap() {
  const auto& m = map->GetMapRef();
  coverageMap.resize(m.GetData().size());
  std::transform(m.GetData().begin(), m.GetData().end(), coverageMap.begin(),
                 [](const Mapping::MapManager::MapT::CellVal& m) {
                   return m != Mapping::MapManager::MapT::FreeVal;
                 });
}

void CoveragePathGenerator::updateCoverageMap(const IntPoint& pt) {
  const auto& m = map->GetMapRef();
  const int vehicleRadiusCells = Constants::VehicleRadius / m.GetResolution();
  for (int x = -vehicleRadiusCells; x <= vehicleRadiusCells; ++x) {
    for (int y = -vehicleRadiusCells; y <= vehicleRadiusCells; ++y) {
      const auto cellCoords = pt + IntPoint{x, y};
      if (m.IsInBounds(cellCoords)) {
        coverageMap[m.GetIdx(cellCoords)] = true;
      }
    }
  }
}

bool CoveragePathGenerator::HasUpdatedPath() const { return false; }

size_t CoveragePathGenerator::getNumCoveredForMove(const IntPoint& pt) const {
  const auto& m = map->GetMapRef();
  const int vehicleRadiusCells = Constants::VehicleRadius / m.GetResolution();
  size_t ret = 0;
  for (int x = -vehicleRadiusCells; x <= vehicleRadiusCells; ++x) {
    for (int y = -vehicleRadiusCells; y <= vehicleRadiusCells; ++y) {
      const auto cellCoords = pt + IntPoint{x, y};
      if (m.IsInBounds(cellCoords) && !coverageMap[m.GetIdx(cellCoords)]) {
        ++ret;
      }
    }
  }
  return ret;
}

std::optional<IntPoint> CoveragePathGenerator::getNextBestMove(
    const IntPoint& pt) const {
  IntPoint bestMove = pt;
  size_t mostUpdated = 0;
  const auto& m = map->GetMapRef();

  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      const auto cellCoords = pt + IntPoint{x, y};
      const size_t numUpdated = getNumCoveredForMove(cellCoords);
      if (numUpdated > mostUpdated &&
          m.IsSafe(cellCoords, Constants::VehicleRadius)) {
        mostUpdated = numUpdated;
        bestMove = cellCoords;
      }
    }
  }

  if (bestMove == pt) {
    return {};
  }
  return bestMove;
}

std::optional<IntPoint> CoveragePathGenerator::getNewStartingPoint(
    const IntPoint& pt, std::vector<IntPoint>& path) const {
  int radius = 1;
  const auto& m = map->GetMapRef();
  while (pt.x + radius < m.GetWidth() || pt.x - radius > 0 ||
         pt.y + radius < m.GetHeight() || pt.y - radius > 0) {
    for (int x = -radius; x <= radius; ++x) {
      const auto newPt1 = checkNewPointForStartPoint(pt, {x, radius}, path);
      if (newPt1.has_value()) {
        return newPt1;
      }

      const auto newPt2 = checkNewPointForStartPoint(pt, {x, -radius}, path);
      if (newPt2.has_value()) {
        return newPt2;
      }
    }
    for (int y = -radius; y <= radius; ++y) {
      const auto newPt1 = checkNewPointForStartPoint(pt, {radius, y}, path);
      if (newPt1.has_value()) {
        return newPt1;
      }

      const auto newPt2 = checkNewPointForStartPoint(pt, {-radius, y}, path);
      if (newPt2.has_value()) {
        return newPt2;
      }
    }
    ++radius;
  }
  return {};
}

std::optional<IntPoint> CoveragePathGenerator::checkNewPointForStartPoint(
    const IntPoint& currPt, const IntPoint& offset,
    std::vector<IntPoint>& path) const {
  const auto& m = map->GetMap();
  IntPoint nxtPt = currPt + offset;
  if (m.IsInBounds(nxtPt) && getNumCoveredForMove(nxtPt) > 0 &&
      m.IsSafe(nxtPt, Constants::VehicleRadius)) {
    const auto sPoint = m.GetPoint(currPt);
    const auto ePoint = m.GetPoint(nxtPt);
    if (!m.IsSafePath(sPoint, ePoint, Constants::VehicleRadius)) {
      AStar astar;
      AStar::Params p{};
      p.planningTime = 1.0;
      p.planningResolution = Constants::VehicleRadius;
      p.vehicleRadius = Constants::VehicleRadius;
      const auto plannedPath = astar.PlanPath(sPoint, ePoint, m, p);
      if (plannedPath.empty() ||
          !m.IsSafePath(ePoint, plannedPath.back(), Constants::VehicleRadius)) {
        std::cout << "Failed to plan path: " << plannedPath.size()
                  << " from: " << sPoint << " to " << ePoint << '\n';
        return {};
      }
      std::transform(plannedPath.begin(), plannedPath.end(),
                     std::back_inserter(path),
                     [m](const auto& pt) { return m.GetCellCoords(pt); });
    }
    return nxtPt;
  }
  return {};
}

}  // namespace Navigation::PathGenerators
