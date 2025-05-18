#pragma once
#include <std_msgs/msg/empty.hpp>

#include "owen_navigation/path_generators/BasePathGenerator.hpp"
namespace Navigation::PathGenerators {

class CoveragePathGenerator : public BasePathGenerator {
  using IntPoint = owen_common::types::BasePoint2D<int>;
  enum class CellState {
    Clean,
    Dirty,
    Planned
  };
  using MapT = Mapping::GridMap<CellState, CellState::Dirty, CellState::Clean, CellState::Planned>;

 public:
  explicit CoveragePathGenerator(rclcpp::Node& n,
                                 std::shared_ptr<Mapping::MapManager> map);
  std::vector<owen_common::types::Point2D> GeneratePath(
      const owen_common::types::Pose2D& pose) override;

  bool HasNewCommand() override;
  [[nodiscard]] bool HasUpdatedPath(const owen_common::types::Pose2D& pose) override;
  bool IsPathComplete(const owen_common::types::Pose2D& pose) const override;

 private:
  void updateCoverageMap(const IntPoint& pt, CellState state);
  [[nodiscard]] std::optional<IntPoint> getNextBestMove(
      const IntPoint& pt) const;
  [[nodiscard]] size_t getNumCoveredForMove(const IntPoint& pt) const;
  void initializeCoverageMap();
  [[nodiscard]] std::optional<IntPoint> getNewStartingPoint(
      const IntPoint& pt, std::vector<IntPoint>& path) const;
  std::optional<IntPoint> checkNewPointForStartPoint(
      const IntPoint& currPt, const IntPoint& offset,
      std::vector<IntPoint>& path) const;

  nav_msgs::msg::OccupancyGrid::UniquePtr getOccupancyGridMsg() const;

 private:
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr startCommandSub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancyGridPub_;
  rclcpp::TimerBase::SharedPtr mapPubTimer_;
  bool hasNewCommand_;
  MapT coverageMap_;

  IntPoint lastEndPt_;
};

}  // namespace Navigation::PathGenerators
