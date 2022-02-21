#ifndef EXPAND_TO_GOAL_COSTMAP_PLUGIN__EXPAND_TO_GOAL_COSTMAP_PLUGIN_HPP_
#define EXPAND_TO_GOAL_COSTMAP_PLUGIN__EXPAND_TO_GOAL_COSTMAP_PLUGIN_HPP_

#include "expand_to_goal_costmap_plugin/visibility_control.h"
#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/subscription.hpp>

namespace expand_to_goal_costmap_plugin
{
using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
class ExpandToGoal : public nav2_costmap_2d::CostmapLayer
{
public:
  ExpandToGoal();

  virtual void onInitialize();
  virtual void updateBounds(double /*robot_x*/, double /* robot_y*/, double /*robot_yaw*/, double* min_x, double* min_y,
                            double* max_x, double* maxY);

  virtual void reset();
  virtual void activate();
  virtual void deactivate();
  virtual void updateCosts(nav2_costmap_2d::Costmap2D&, int, int, int, int);

  virtual bool isClearable()
  {
    return false;
  }
  virtual void matchSize();

  void goalCallback(geometry_msgs::msg::PoseStamped::UniquePtr msg);

private:
  /**
   * @brief Get parameters of layer
   */
  void getParameters();

  /**
   * @brief Process a new map coming from a topic
   */
  void processMap(const nav_msgs::msg::OccupancyGrid& new_map);

  /**
   * @brief  Callback to update the costmap's map from the map_server
   * @param new_map The map to put into the costmap. The origin of the new
   * map along with its size will determine what parts of the costmap's
   * static map are overwritten.
   */
  void incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr new_map);
  /**
   * @brief Callback to update the costmap's map from the map_server (or SLAM)
   * with an update in a particular area of the map
   */
  void incomingUpdate(map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update);

  /**
   * @brief Interpret the value in the static map given on the topic to
   * convert into costs for the costmap to utilize
   */
  unsigned char interpretValue(unsigned char value);

  void resizeMapWithGoal(unsigned int size_x, unsigned int size_y, double resolution, double origin_x, double origin_y);

private:
  std::string global_frame_;  ///< @brief The global frame for the costmap
  std::string map_frame_;     /// @brief frame that map is located in

  bool has_updated_data_{ false };

  unsigned int x_{ 0 };
  unsigned int y_{ 0 };
  unsigned int width_{ 0 };
  unsigned int height_{ 0 };

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr map_update_sub_;

  // Parameters
  std::string map_topic_;
  bool map_subscribe_transient_local_;
  bool subscribe_to_updates_;
  bool track_unknown_space_;
  bool use_maximum_;
  unsigned char lethal_threshold_;
  unsigned char unknown_cost_value_;
  bool trinary_costmap_;
  bool map_received_{ false };
  tf2::Duration transform_tolerance_;
  std::atomic<bool> update_in_progress_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_buffer_;
  unsigned int newSizeX{ 0 };
  unsigned int newSizeY{ 0 };
  double newOriginX{ 0 };
  double newOriginY{ 0 };
  double x{ 0 };
  double y{ 0 };
  bool hasData;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalSubscriber;
};

}  // namespace expand_to_goal_costmap_plugin

#endif  // EXPAND_TO_GOAL_COSTMAP_PLUGIN__EXPAND_TO_GOAL_COSTMAP_PLUGIN_HPP_
