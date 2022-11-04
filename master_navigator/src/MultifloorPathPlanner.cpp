#include "master_navigator/MultifloorPathPlanner.hpp"
#include <roomba_msgs/msg/detail/multifloor_path__struct.hpp>
#include <yaml-cpp/yaml.h>

MultifloorPathPlanner::MultifloorPathPlanner(rclcpp::Node::SharedPtr _node) : node(_node)
{
  node->declare_parameter("/maps", rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
  std::vector<std::string> map_names;
  node->get_parameter("/maps", map_names);

  node->declare_parameter("/map_node_file", rclcpp::ParameterType::PARAMETER_STRING);
  std::string map_node_file;
  node->get_parameter("/map_node_file", map_node_file);

  this->read_map_nodes(map_node_file);
}

void MultifloorPathPlanner::read_map_nodes(const std::string& map_node_file)
{
  YAML::Node top = YAML::LoadFile(map_node_file);

  for (const auto& n : top)
  {
    int id = n.first.as<int>();
    if (map_nodes.count(id) > 0)
    {
      RCLCPP_WARN(node->get_logger(), "Got duplicate node id %d", id);
      continue;
    }

    map_nodes[id].node_id = id;
    for (const auto& conn : n.second["connections"])
    {
      map_nodes[id].connections.push_back({ conn.second[id].as<int>(), conn.second["cost"].as<double>() });
    }

    map_nodes[id].point.floor_id.data = n.second["floor_id"].as<std::string>();
    map_nodes[id].point.point.x = n.second["x"].as<double>();
    map_nodes[id].point.point.y = n.second["y"].as<double>();
  }
}

roomba_msgs::msg::MultifloorPath MultifloorPathPlanner::plan_path(roomba_msgs::msg::MultifloorPoint _destination,
                                                                  std::string _current_floor,
                                                                  geometry_msgs::msg::Point _current_position)
{
  destination = _destination;
  current_position.floor_id.data = _current_floor;
  current_position.point = _current_position;
}
