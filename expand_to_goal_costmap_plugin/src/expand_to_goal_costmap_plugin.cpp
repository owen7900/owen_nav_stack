#include "expand_to_goal_costmap_plugin/expand_to_goal_costmap_plugin.hpp"
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <rclcpp/logging.hpp>

namespace expand_to_goal_costmap_plugin
{
using std::placeholders::_1;

ExpandToGoal::ExpandToGoal()
{
}

void
ExpandToGoal::onInitialize()
{
    this->goalSubscriber = this->rclcpp_node_->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&ExpandToGoal::goalCallback, this, _1));
    this->hasData = false;
}

void
ExpandToGoal::goalCallback(geometry_msgs::msg::PoseStamped::UniquePtr msg)
{
    if(msg)
    {
        this->x = msg->pose.position.x;
        this->y = msg->pose.position.y;
        this->hasData = true;



        int mx, my;
        const auto costMap = this->layered_costmap_->getCostmap();
        costMap->worldToMapNoBounds(this->x, this->y, mx, my);
        RCLCPP_INFO_STREAM(this->logger_, "ExpandToGoal got mxy" << mx << ", " << my );
        if(mx > 0 && my > 0)
        {
            RCLCPP_INFO_STREAM(this->logger_, "ExpandToGoal resizing map to " << mx<< ", " << my );
            this->layered_costmap_->resizeMap(std::max(mx, static_cast<int>(costMap->getSizeInCellsX())),
                        std::max(my, static_cast<int>(costMap->getSizeInCellsY())),
                        costMap->getResolution(), costMap->getOriginX(), costMap->getOriginY());
        }
        else
        {
            int newSizeX = costMap->getSizeInCellsX();
            int newSizeY = costMap->getSizeInCellsY();
            double newOriginX = costMap->getOriginX();
            double newOriginY = costMap->getOriginY();
            if(mx > newSizeX)
            {
                newSizeX = mx;
            }
            else if(mx < 0)
            {
                newOriginX = this->x;
                newSizeX += std::abs(mx);
            }

            if(my > newSizeY)
            {
                newSizeY = my;
            }
            else if(mx < 0)
            {
                newOriginY = this->y;
                newSizeY += std::abs(my);
            }
            RCLCPP_INFO_STREAM(this->logger_, "ExpandToGoal resizing map to " << newSizeX << ", " << newSizeY << " with origin " << newOriginX << ", " << newOriginY);
            this->layered_costmap_->resizeMap(newSizeX, newSizeY, costMap->getResolution(), newOriginX, newOriginY);

        }
    }
}

}  // namespace expand_to_goal_costmap_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(expand_to_goal_costmap_plugin::ExpandToGoal,  nav2_costmap_2d::Layer)
