#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <roomba_msgs/msg/detail/multifloor_path__struct.hpp>
#include <string>
#include <vector>
#include <unordered_map>

#include "roomba_msgs/msg/multifloor_point.hpp"

struct MapNode
{
  int node_id;
  roomba_msgs::msg::MultifloorPoint point;
  std::vector<std::pair<int, double>> connections;
};

/// This class also moves current position and destination to their closest map locations
class MultifloorPathPlanner
{
public:
  MultifloorPathPlanner(rclcpp::Node::SharedPtr _node);

  roomba_msgs::msg::MultifloorPath plan_path(roomba_msgs::msg::MultifloorPoint destination,
                                             roomba_msgs::msg::MultifloorPoint current_position);

private:
  void read_map_nodes(const std::string& map_file);

  roomba_msgs::msg::MultifloorPath plan_path_on_same_floor(const roomba_msgs::msg::MultifloorPoint& start,
                                                           const roomba_msgs::msg::MultifloorPoint& end);

  int get_closest_node_id(const roomba_msgs::msg::MultifloorPoint& point) const;

private:
  rclcpp::Node::SharedPtr node;
  std::unordered_map<int, MapNode> map_nodes;
};
