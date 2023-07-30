#include "owen_navigation/path_generators/BaseNavigator.hpp"

namespace Navigation::PathGenerators {

class RRTNavigator : public BaseNavigator {
  template <typename P>
  friend class PlannerTester;
  using Point2D = owen_common::types::Point2D;
  struct Parameters {
    bool doOptimize;
  };

 public:
  struct Node {
    Point2D point;
    size_t parent;
    double cost;
    std::vector<size_t> children;
  };

 public:
  RRTNavigator(rclcpp::Node& node,
               const std::shared_ptr<Mapping::MapManager>& map);

  std::vector<Point2D> GeneratePath(
      const owen_common::types::Pose2D& pose) override;

  bool HasUpdatedPath() const override;

 private:
  void addNode(const owen_common::types::Point2D& pt, size_t parent,
               double cost);

  bool isPointDestination(const Point2D& pt) const;

  Point2D randomlySamplePoint();

 private:
  Parameters rrtParams;
  std::vector<Node> nodes;
};

}  // namespace Navigation::PathGenerators
