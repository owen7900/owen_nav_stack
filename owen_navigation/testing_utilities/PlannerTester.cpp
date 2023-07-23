

#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/utilities.hpp>

#include "owen_navigation/path_generators/AStarNavigator.hpp"
#include "owen_navigation/path_generators/RRTNavigator.hpp"

using Point2D = owen_common::types::Point2D;
using Pose2D = owen_common::types::Pose2D;
using Cell = Navigation::Mapping::Map::Cell;

namespace Navigation::PathGenerators {
class PlannerTester {
 public:
  PlannerTester(rclcpp::Node& n, const std::shared_ptr<Mapping::MapManager>& m)
      : nav(n, m), map(m) {}

  std::vector<Point2D> PlanPath(const Pose2D& start, const Point2D& end) {
    nav.destination.SetData(end);
    return nav.GeneratePath(start);
  }

  void AddObstacle(const Cell& c) { map->GetMapRef().UpdateMap(c); }
  void AddObstacle(const std::vector<Cell>& cells) {
    map->GetMapRef().UpdateMap(cells);
  }

  void DrawTree(cv::Mat& m, const cv::Vec3b& color = cv::Vec3b{0, 0, 0}) {
    // for (const auto& n : nav.prevNodes) {
    //   const auto xy1 = map->GetMap().GetCellCoords(n.first.getPoint());
    //   const auto xy2 = map->GetMap().GetCellCoords(n.second.getPoint());
    //   cv::line(m, cv::Point(xy1.x, xy1.y), cv::Point(xy2.x, xy2.y), color);
    // }
    for (const auto& n : nav.nodes) {
      for (const auto& i : n.children) {
        const auto xy1 = map->GetMap().GetCellCoords(n.point);
        const auto xy2 = map->GetMap().GetCellCoords(nav.nodes.at(i).point);
        cv::line(m, cv::Point(xy1.x, xy1.y), cv::Point(xy2.x, xy2.y), color);
      }
    }
  };

 private:
  RRTNavigator nav;
  std::shared_ptr<Mapping::MapManager> map;
};
}  // namespace Navigation::PathGenerators

void printPath(const std::vector<Point2D>& path) {
  std::cout << "Got Path: ";
  for (const auto& pt : path) {
    std::cout << pt << ", ";
  }
  std::cout << std::endl;
}
bool arePathsSame(const std::vector<Point2D>& p1,
                  const std::vector<Point2D>& p2) {
  if (p1.size() != p2.size()) {
    return false;
  }

  for (size_t i = 0; i < p1.size(); ++i) {
    if (p1[i] != p2[i]) {
      return false;
    }
  }

  return true;
}

cv::Mat getImage(const std::shared_ptr<Navigation::Mapping::MapManager>& mngr) {
  const auto& map = mngr->GetMap();
  cv::Mat m(map.GetHeight(), map.GetWidth(), CV_8UC3);
  for (size_t i = 0; i < map.GetWidth(); ++i) {
    for (size_t j = 0; j < map.GetHeight(); ++j) {
      m.at<cv::Vec3b>(j, i) =
          map.IsOccupied(Navigation::Mapping::Map::IntPoint{i, j})
              ? cv::Vec3b(0, 0, 255)
              : cv::Vec3b(255, 255, 255);
    }
  }
  return m;
}

void drawPath(cv::Mat& mat, const std::vector<Point2D>& path,
              const std::shared_ptr<Navigation::Mapping::MapManager>& mngr,
              const cv::Vec3b& color = cv::Vec3b{0, 0, 0}) {
  const auto& map = mngr->GetMap();
  auto prevPt = path.front();
  const size_t numPts = path.size();
  for (size_t i = 1; i < numPts; ++i) {
    const auto xy1 = map.GetCellCoords(prevPt);
    const auto xy2 = map.GetCellCoords(path[i]);
    cv::line(mat, cv::Point(xy1.x, xy1.y), cv::Point(xy2.x, xy2.y), color);
    prevPt = path[i];
  }
}

void drawPt(cv::Mat& mat, const Point2D& point,
            const std::shared_ptr<Navigation::Mapping::MapManager>& mngr,
            const cv::Vec3b& color = cv::Vec3b{0, 0, 0}) {
  const auto& map = mngr->GetMap();
  const auto c = map.GetCellCoords(point);
  cv::circle(mat, {c.x, c.y}, 5, color, cv::FILLED);
}

void addRandomObstacles(
    const std::shared_ptr<Navigation::Mapping::MapManager>& map, size_t seed) {
  if (seed == 0) {
    seed = std::chrono::system_clock::now().time_since_epoch().count();
  }
  std::cout << "Using seed: " << seed << std::endl;
  srand(seed);
  auto& mapRef = map->GetMapRef();
  mapRef.ResizeToPoint({100, 100});
  std::vector<Navigation::Mapping::Map::Cell> updates;

  size_t numObs = rand() % 40;
  for (size_t i = 0; i < numObs; ++i) {
    Navigation::Mapping::Map::Cell c;
    c.state = true;
    owen_common::types::Point2D minPt{
        rand() % mapRef.GetWidth() * mapRef.GetResolution(),
        rand() % mapRef.GetHeight() * mapRef.GetResolution()};
    owen_common::types::Point2D maxPt =
        minPt +
        owen_common::types::Point2D{rand() % 400 * mapRef.GetResolution(),
                                    rand() % 400 * mapRef.GetResolution()};
    c.bounds = {minPt, maxPt};

    updates.push_back(c);
  }
  mapRef.UpdateMap(updates);
}

struct Params {
  size_t seed;
};

Params parseParams(const std::vector<std::string>& args) {
  Params p{};
  for (size_t i = 0; i < args.size(); ++i) {
    if (args.at(i) == "-s" && args.size() > i + 1) {
      try {
        p.seed = std::stoul(args.at(++i));
      } catch (std::exception& e) {
        std::cerr << "Got " << e.what() << " from: " << args.at(i) << std::endl;
      }
    }
  }

  return p;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  const auto params = parseParams(rclcpp::remove_ros_arguments(argc, argv));
  rclcpp::Node n("planner_tester");
  n.declare_parameter("planning_resolution", 0.05);
  auto map = std::make_shared<Navigation::Mapping::MapManager>(n);
  Navigation::PathGenerators::PlannerTester tester(n, map);
  const auto t1 = std::chrono::system_clock::now();
  const auto path1 = tester.PlanPath({0, 0, 0}, {100, 100});
  const std::chrono::duration<double> d1 =
      std::chrono::system_clock::now() - t1;
  std::cout << "Planned P1 in: " << std::fixed << d1.count() << "s. Was "
            << path1.size() << " points long" << std::endl;

  addRandomObstacles(map, params.seed);

  const auto t2 = std::chrono::system_clock::now();
  const auto path2 = tester.PlanPath({0, 0, 0}, {100, 100});
  const std::chrono::duration<double> d2 =
      std::chrono::system_clock::now() - t2;
  std::cout << "Planned P2 in: " << std::fixed << d2.count() << "s. Was "
            << path2.size() << " points long " << std::endl;

  auto m = getImage(map);
  drawPath(m, path1, map);
  drawPath(m, path2, map, {255, 0, 0});
  // tester.DrawTree(m, {0, 255, 0});
  drawPt(m, {100, 100}, map, {0, 255, 0});
  cv::imshow("MAP", m);
  while (cv::waitKey(0) != 'q') {
    ;
  }

  rclcpp::shutdown();
}
