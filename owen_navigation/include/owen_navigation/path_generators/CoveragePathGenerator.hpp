#pragma once
#include <std_msgs/msg/empty.hpp>

#include "owen_navigation/path_generators/BasePathGenerator.hpp"
namespace Navigation::PathGenerators {

class CoveragePathGenerator : public BasePathGenerator {
  using IntPoint = owen_common::types::BasePoint2D<int>;

 public:
  explicit CoveragePathGenerator(rclcpp::Node& n,
                                 std::shared_ptr<Mapping::MapManager> map);
  std::vector<owen_common::types::Point2D> GeneratePath(
      const owen_common::types::Pose2D& pose) override;

  bool HasNewCommand() override;
  [[nodiscard]] bool HasUpdatedPath() const override;

 private:
  void updateCoverageMap(const IntPoint& pt);
  [[nodiscard]] std::optional<IntPoint> getNextBestMove(
      const IntPoint& pt) const;
  [[nodiscard]] size_t getNumCoveredForMove(const IntPoint& pt) const;
  void initializeCoverageMap();
  [[nodiscard]] std::optional<IntPoint> getNewStartingPoint(
      const IntPoint& pt, std::vector<IntPoint>& path) const;
  std::optional<IntPoint> checkNewPointForStartPoint(
      const IntPoint& currPt, const IntPoint& offset,
      std::vector<IntPoint>& path) const;

 private:
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr startCommandSub_;
  bool hasNewCommand_;
  std::vector<bool> coverageMap;
};

}  // namespace Navigation::PathGenerators
