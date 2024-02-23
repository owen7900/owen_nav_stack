
#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/utilities.hpp>

#include "owen_navigation/mapping/obstacle_sources/ObstacleSourceFactory.hpp"
#include "owen_navigation/path_generators/CoveragePathGenerator.hpp"
using Point2D = owen_common::types::Point2D;
using Pose2D = owen_common::types::Pose2D;
using Cell = Navigation::Mapping::MapManager::MapT::Cell;
using namespace Navigation::PathGenerators;

cv::Mat getImage(const std::shared_ptr<Navigation::Mapping::MapManager>& mngr) {
  const auto& map = mngr->GetMap();
  cv::Mat m(map.GetHeight(), map.GetWidth(), CV_8UC3);
  for (size_t i = 0; i < map.GetWidth(); ++i) {
    for (size_t j = 0; j < map.GetHeight(); ++j) {
      Navigation::Mapping::MapManager::MapT::IntPoint pt{static_cast<int>(i),
                                                         static_cast<int>(j)};
      m.at<cv::Vec3b>(j, i) =
          !map.IsFree(pt) ? cv::Vec3b(0, 0, map.GetData().at(map.GetIdx(pt)))
                          : cv::Vec3b(255, 255, 255);
    }
  }
  return m;
}

void drawPath(cv::Mat& mat, const std::vector<Point2D>& path,
              const std::shared_ptr<Navigation::Mapping::MapManager>& mngr,
              const cv::Vec3b& color = cv::Vec3b{0, 0, 0}) {
  if (path.empty()) {
    return;
  }
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
  cv::circle(mat, {c.x, c.y}, 0.15 / map.GetResolution(), color, cv::FILLED);
}
#define SMALL_MAP
#ifdef SMALL_MAP

void addRandomObstacles(
    const std::shared_ptr<Navigation::Mapping::MapManager>& map) {
  auto& mapRef = map->GetMapRef();
  mapRef.ResizeToPoint({100, 100});
  mapRef.Fill(Navigation::Mapping::MapManager::MapT::FreeVal);
  std::vector<Navigation::Mapping::MapManager::MapT::Cell> updates;

  std::cout << "Min: " << mapRef.GetOrigin() << " max: " << mapRef.GetMaxPoint()
            << "\n";

  size_t numObs = rand() % 400;
  for (size_t i = 0; i < numObs; ++i) {
    Navigation::Mapping::MapManager::MapT::Cell c;
    c.state = Navigation::Mapping::MapManager::MapT::OccupiedVal;
    const auto origin = mapRef.GetOrigin();
    owen_common::types::Point2D minPt =
        origin +
        Point2D{rand() % (mapRef.GetWidth() - 1) * mapRef.GetResolution(),
                rand() % (mapRef.GetHeight() - 1) * mapRef.GetResolution()};
    minPt.x = std::max(minPt.x, origin.x);
    minPt.y = std::max(minPt.y, origin.y);
    owen_common::types::Point2D maxPt =
        minPt + owen_common::types::Point2D{1, 1};
    const auto fMaxPt = mapRef.GetMaxPoint();
    maxPt.x = std::min(fMaxPt.x, maxPt.x);
    maxPt.y = std::min(fMaxPt.y, maxPt.y);
    if (!mapRef.IsInBounds(minPt) || !mapRef.IsInBounds(maxPt)) {
      std::cout << "Out of bounds: " << minPt << " max: " << maxPt << "\n";
    }
    c.bounds = {minPt, maxPt};

    updates.push_back(c);
  }
  mapRef.UpdateMap(updates);
}
#else
void addRandomObstacles(
    const std::shared_ptr<Navigation::Mapping::MapManager>& map) {
  auto& mapRef = map->GetMapRef();
  mapRef.ResizeToPoint({100, 100});
  mapRef.Fill(Navigation::Mapping::MapManager::MapT::FreeVal);
  std::vector<Navigation::Mapping::MapManager::MapT::Cell> updates;

  size_t numObs = rand() % 40;
  for (size_t i = 0; i < numObs; ++i) {
    Navigation::Mapping::MapManager::MapT::Cell c;
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
#endif
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
  n.declare_parameter("planning_resolution", 1.0);
  n.declare_parameter("max_planning_time", 1.0);
  auto map = std::make_shared<Navigation::Mapping::MapManager>(n);
  size_t seed = params.seed;
  if (seed == 0) {
    seed = std::chrono::system_clock::now().time_since_epoch().count();
  }
  std::cout << "Using seed: " << seed << std::endl;
  srand(seed);
  addRandomObstacles(map);
  CoveragePathGenerator gen(n, map);

  const auto time = std::chrono::system_clock::now();
  const auto path = gen.GeneratePath({0, 0, 0});
  const std::chrono::duration<double> dur =
      std::chrono::system_clock::now() - time;

  std::cout << "Took: " << dur.count()
            << " to plan path of size: " << path.size() << std::endl;

  auto m = getImage(map);
  drawPath(m, path, map, {0, 255, 0});
  drawPt(m, {0.0, 0.}, map, {255, 0, 0});
  cv::imshow("MAP", m);
  while (cv::waitKey(0) != 'q' && rclcpp::ok()) {
    ;
  }

  rclcpp::shutdown();
  return 0;
}
