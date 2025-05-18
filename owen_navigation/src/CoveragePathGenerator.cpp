#include "owen_navigation/path_generators/CoveragePathGenerator.hpp"

#include <tf2/LinearMath/Quaternion.h>

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
        coverageMap_.GetData().clear();
        initializeCoverageMap();
      });
  occupancyGridPub_ =
      n.create_publisher<nav_msgs::msg::OccupancyGrid>("cleaning_coverage", 1);
  mapPubTimer_ = n.create_wall_timer(
      std::chrono::duration<double>(1.0),
      [this] { occupancyGridPub_->publish(getOccupancyGridMsg()); });
  initializeCoverageMap();
}

std::vector<owen_common::types::Point2D> CoveragePathGenerator::GeneratePath(
    const owen_common::types::Pose2D& pose) {
  const auto rbtCellCoords = coverageMap_.GetCellCoords({pose.x, pose.y});
  std::vector<IntPoint> plan;
  std::optional<IntPoint> currPos{rbtCellCoords};
  for (auto& c : coverageMap_.GetData()) {
    if (c == CellState::Planned) {
      c = CellState::Dirty;
    }
  }
  while (currPos.has_value() && rclcpp::ok() && plan.size() < 10) {
    plan.push_back(currPos.value());
    updateCoverageMap(currPos.value_or(plan.back()), CellState::Planned);
    currPos = getNextBestMove(currPos.value_or(plan.back()));
  }
  if (plan.size() <= 1 || plan.back().distanceFromPoint(rbtCellCoords) *
                                  coverageMap_.GetResolution() <
                              (Constants::VehicleRadius * 2)) {
    plan.clear();
    plan.push_back(rbtCellCoords);
    // RCLCPP_INFO(rclcpp::get_logger("coverage_path_generator"),
    //             "Traversing to new start location");
    currPos = getNewStartingPoint(plan.back(), plan);
    if (currPos.has_value()) {
      plan.push_back(currPos.value());
    } else {
      RCLCPP_WARN(rclcpp::get_logger("coverage_path_generator"),
                  "Unable to calculate new start location");
    }
  }

  if (!plan.empty()) {
    lastEndPt_ = plan.back();
  }

  std::vector<owen_common::types::Point2D> ret(plan.size());
  std::transform(plan.begin(), plan.end(), ret.begin(),
                 [this](const auto& pt) { return coverageMap_.GetPoint(pt); });
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
  coverageMap_ = MapT(m.GetOrigin(), m.GetMaxPoint(), m.GetResolution());
  std::transform(
      m.GetData().begin(), m.GetData().end(), coverageMap_.GetData().begin(),
      [](const Mapping::MapManager::MapT::CellVal& cVal) {
        return cVal != Mapping::MapManager::MapT::FreeVal ? CellState::Clean
                                                          : CellState::Dirty;
      });
}

void CoveragePathGenerator::updateCoverageMap(const IntPoint& pt,
                                                  CellState state) {
  const int vehicleRadiusCells =
      std::ceil(Constants::VehicleRadius / coverageMap_.GetResolution());
  for (int x = -vehicleRadiusCells; x <= vehicleRadiusCells; ++x) {
    for (int y = -vehicleRadiusCells; y <= vehicleRadiusCells; ++y) {
      const auto cellCoords = pt + IntPoint{x, y};
      if (coverageMap_.IsInBounds(cellCoords) &&
          coverageMap_[cellCoords] != CellState::Clean) {
        if (state == CellState::Clean)
        coverageMap_[cellCoords] = state;
      }
    }
  }
}

bool CoveragePathGenerator::HasUpdatedPath(
    const owen_common::types::Pose2D& pose) {
  const auto& m = map->GetMapRef();
  const auto currPos = coverageMap_.GetCellCoords({pose.x, pose.y});
  updateCoverageMap(currPos, CellState::Clean);
  return lastEndPt_.distanceFromPoint(currPos) * coverageMap_.GetResolution() <
         (Constants::VehicleRadius * 2);
}

size_t CoveragePathGenerator::getNumCoveredForMove(const IntPoint& pt) const {
  const int vehicleRadiusCells =
      std::ceil(Constants::VehicleRadius / coverageMap_.GetResolution());
  size_t ret = 0;
  for (int x = -vehicleRadiusCells; x <= vehicleRadiusCells; ++x) {
    for (int y = -vehicleRadiusCells; y <= vehicleRadiusCells; ++y) {
      const auto cellCoords = pt + IntPoint{x, y};
      if (coverageMap_.IsInBounds(cellCoords) &&
          coverageMap_[cellCoords] == CellState::Dirty) {
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
  while (pt.x + radius < coverageMap_.GetWidth() || pt.x - radius > 0 ||
         pt.y + radius < coverageMap_.GetHeight() || pt.y - radius > 0) {
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
  if (m.IsInBounds(nxtPt) && getNumCoveredForMove(nxtPt) > 5 &&
      m.IsSafe(nxtPt, Constants::VehicleRadius)) {
    const auto sPoint = m.GetPoint(currPt);
    const auto ePoint = m.GetPoint(nxtPt);
    if (!m.IsSafePath(sPoint, ePoint, Constants::VehicleRadius)) {
      AStar astar;
      AStar::Params p{};
      p.planningTime = 1.0;
      p.planningResolution = Constants::VehicleRadius / 2;
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

bool CoveragePathGenerator::IsPathComplete(
    const owen_common::types::Pose2D& pose) const {
  const auto currPose = coverageMap_.GetCellCoords({pose.x, pose.y});
  std::vector<IntPoint> path;
  return !getNextBestMove(currPose).has_value() &&
         !getNewStartingPoint(currPose, path).has_value();
}

nav_msgs::msg::OccupancyGrid::UniquePtr
CoveragePathGenerator::getOccupancyGridMsg() const {
  auto ret = std::make_unique<nav_msgs::msg::OccupancyGrid>();

  ret->header.frame_id = "map";

  ret->info.width = coverageMap_.GetWidth();
  ret->info.height = coverageMap_.GetHeight();
  ret->info.resolution = coverageMap_.GetResolution();

  auto origin = coverageMap_.GetOrigin();
  ret->info.origin.position.x = origin.x;
  ret->info.origin.position.y = origin.y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  ret->info.origin.orientation.x = q.x();
  ret->info.origin.orientation.y = q.y();
  ret->info.origin.orientation.z = q.z();
  ret->info.origin.orientation.w = q.w();

  ret->data.resize(coverageMap_.GetData().size());
  std::transform(coverageMap_.GetData().begin(), coverageMap_.GetData().end(),
                 ret->data.begin(), [](const CellState& s) {
                   switch (s) {
                     case CellState::Clean:
                       return 255;
                     case CellState::Dirty:
                       return 0;
                     default:
                     case CellState::Planned:
                       return 100;
                   }
                 });

  return ret;
}

}  // namespace Navigation::PathGenerators
