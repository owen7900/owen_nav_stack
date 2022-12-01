//
// Created by kevin on 11/2/22.
//

#ifndef SEMANTIC_COSTMAP_PLUGIN__SEMANTIC_COSTMAP_PLUGIN_HPP_
#define SEMANTIC_COSTMAP_PLUGIN__SEMANTIC_COSTMAP_PLUGIN_HPP_

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "std_msgs/msg/string.hpp"
#include "roomba_msgs/msg/multifloor_rectangle.hpp"
#include <rclcpp/subscription.hpp>

namespace semantic_costmap_plugin {

class SemanticMap : public nav2_costmap_2d::Layer {
    public:
        SemanticMap();
        virtual void onInitialize();
        virtual void updateBounds(double /*robot_x*/, double /* robot_y*/, double /*robot_yaw*/, double* min_x, double* min_y,
                                  double* max_x, double* maxY);
        virtual void onFootprintChanged();
        virtual void reset(){};
        virtual void activate(){};
        virtual void deactivate(){};
        virtual void updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j);
        virtual bool isClearable() { return false; }

    private:
        void setup();
        void floorCallback(const std_msgs::msg::String::SharedPtr msg);


    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr floor_sub_;
        std::string floor_;
        double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
        bool need_recalculation_;
        std::vector<roomba_msgs::msg::MultifloorRectangle> no_pass_rects_;
        std::vector<std::string> no_pass_params_;
    };

} // namespace semantic_costmap_plugin


#endif //SEMANTIC_COSTMAP_PLUGIN__SEMANTIC_COSTMAP_PLUGIN_HPP_
