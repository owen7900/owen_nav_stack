#include "master_navigator/MultifloorPathPlanner.hpp"
#include <roomba_msgs/msg/detail/multifloor_path__struct.hpp>
#include <yaml-cpp/yaml.h>
#include <queue>

#include "owen_common/PriorityQueue.hpp"

MultifloorPathPlanner::MultifloorPathPlanner(rclcpp::Node::SharedPtr _node) : node(_node)
{
  node->declare_parameter("/map_node_file", rclcpp::ParameterType::PARAMETER_STRING);
  std::string map_node_file;
  if (!node->get_parameter("/map_node_file", map_node_file))
  {
    map_node_file = "/home/owen/owen_ws/src/nav_stack/owen_bringup/config/map_nodes.yaml";
  }

  this->read_map_nodes(map_node_file);
}

void MultifloorPathPlanner::read_map_nodes(const std::string& map_node_file)
{
  RCLCPP_INFO_STREAM(node->get_logger(), "Parsing " << map_node_file);
  YAML::Node top = YAML::LoadFile(map_node_file);
  RCLCPP_INFO_STREAM(node->get_logger(), "Parsed " << map_node_file);

  for (const auto& n : top)
  {
    const int id = n.first.as<int>();
    map_nodes[id].node_id = id;

    for (const auto& conn : n.second["connections"])
    {
      const int connId = conn["id"].as<int>();
      map_nodes[id].connections.push_back({ connId, conn["cost"].as<double>() });
      map_nodes[connId].connections.push_back({ id, conn["cost"].as<double>() });
    }

    map_nodes[id].point.floor_id.data = n.second["floor_id"].as<std::string>();
    map_nodes[id].point.point.x = n.second["x"].as<double>();
    map_nodes[id].point.point.y = n.second["y"].as<double>();
  }

  for (const auto& n : map_nodes)
  {
    std::cout << "id: " << n.first << " Pos: {" << n.second.point.point.x << ", " << n.second.point.point.y << ", "
              << n.second.point.floor_id.data << "} Conns: ";
    for (const auto& i : n.second.connections)
    {
      std::cout << "{" << i.first << ", " << i.second << "}, ";
    }

    std::cout << std::endl;
  }
}

double get_sq_distance_between_points(const geometry_msgs::msg::Point& pt1, const geometry_msgs::msg::Point& pt2)
{
  return std::pow(pt1.x - pt2.x, 2.0) + std::pow(pt1.y - pt2.y, 2.0) + std::pow(pt1.z - pt2.z, 2.0);
}

int MultifloorPathPlanner::get_closest_node_id(const roomba_msgs::msg::MultifloorPoint& point) const
{
  double min_dist = std::numeric_limits<double>::max();
  int ret = -1;
  for (const auto& m : map_nodes)
  {
    if (m.second.point.floor_id == point.floor_id)
    {
      const double dist = get_sq_distance_between_points(m.second.point.point, point.point);
      if (dist < min_dist)
      {
        ret = m.first;
        min_dist = dist;
      }
    }
  }

  return ret;
}

roomba_msgs::msg::MultifloorPath MultifloorPathPlanner::plan_path(roomba_msgs::msg::MultifloorPoint destination,
                                                                  roomba_msgs::msg::MultifloorPoint current_position)
{
  const int start_node = this->get_closest_node_id(current_position);
  const int end_node = this->get_closest_node_id(destination);

  if (start_node == -1 || end_node == -1)
  {
    RCLCPP_ERROR(node->get_logger(), "Could not find start or end in node graph");
    return roomba_msgs::msg::MultifloorPath();
  }

  std::unordered_map<int, int> prev;

  PriorityQueue<int> queue;
  queue.insert(start_node, 0.0);

  for (const auto& m : map_nodes)
  {
    if (m.first == start_node)
    {
      continue;
    }
    prev[m.first] = -1;
    queue.insert(m.first, std::numeric_limits<double>::max());
  }
  queue.update();

  while (!queue.empty())
  {
    const auto id = queue.top();
    if (id == end_node)
    {
      break;
    }
    queue.pop_back();
    const auto& node = map_nodes[id];
    const double curr_dist = queue.get_weight(id);

    for (const auto& neighbour : node.connections)
    {
      const double alt = curr_dist + neighbour.second;
      if (alt < queue.get_weight(neighbour.first))
      {
        queue.update_weight(neighbour.first, alt);
        prev[neighbour.first] = id;
      }
    }
    queue.update();
  }

  roomba_msgs::msg::MultifloorPath ret;

  int i = end_node;
  while (i != start_node)
  {
    ret.points.push_back(map_nodes[i].point);
    i = prev[i];
  }

  ret.points.push_back(map_nodes[start_node].point);

  std::reverse(ret.points.begin(), ret.points.end());
  return ret;
}
