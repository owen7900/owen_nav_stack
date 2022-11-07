//
// Created by kevin on 11/2/22.
//

#ifndef SEMANTIC_COSTMAP_PLUGIN__SEMANTIC_COSTMAP_PLUGIN_HPP_
#define SEMANTIC_COSTMAP_PLUGIN__SEMANTIC_COSTMAP_PLUGIN_HPP_

#include "semantic_costmap_plugin/visibility_control.h"
#include "owen_common/shapes.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include <rclcpp/subscription.hpp>

namespace semantic_costmap_plugin {
    using nav2_costmap_2d::FREE_SPACE;
    using nav2_costmap_2d::LETHAL_OBSTACLE;
    using nav2_costmap_2d::NO_INFORMATION;

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
        void loadConfig();

    private:
        double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
        bool need_recalculation_;
        std::vector<owen_common::CostRect> no_pass_rects_;
        std::vector<owen_common::CostRect> pass_rects_;
        std::vector<std::string> features_;
    };

} // namespace semantic_costmap_plugin


#endif //SEMANTIC_COSTMAP_PLUGIN__SEMANTIC_COSTMAP_PLUGIN_HPP_
