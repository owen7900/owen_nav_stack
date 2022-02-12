#ifndef EXPAND_TO_GOAL_COSTMAP_PLUGIN__EXPAND_TO_GOAL_COSTMAP_PLUGIN_HPP_
#define EXPAND_TO_GOAL_COSTMAP_PLUGIN__EXPAND_TO_GOAL_COSTMAP_PLUGIN_HPP_

#include "expand_to_goal_costmap_plugin/visibility_control.h"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/subscription.hpp>

namespace expand_to_goal_costmap_plugin
{

class ExpandToGoal : public nav2_costmap_2d::Layer
{
public:
  ExpandToGoal();

  virtual void onInitialize();
  virtual void updateBounds(
    double, double, double,  double *,
    double *,
    double *,
    double *){};

  virtual void reset()
  {
    return;
  }

  virtual void updateCosts(nav2_costmap_2d::Costmap2D & , int , int ,
  int ,
  int ){};

  virtual bool isClearable() {return false;}

  void goalCallback(geometry_msgs::msg::PoseStamped::UniquePtr msg);
private:
  double x;
  double y;
  bool hasData;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalSubscriber;

};

}  // namespace expand_to_goal_costmap_plugin

#endif  // EXPAND_TO_GOAL_COSTMAP_PLUGIN__EXPAND_TO_GOAL_COSTMAP_PLUGIN_HPP_
