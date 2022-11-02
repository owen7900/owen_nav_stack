//
// Created by kevin on 11/1/22.
//

#ifndef NAV_STACK_SEMANTICMAP_HPP
#define NAV_STACK_SEMANTICMAP_HPP

#include "semantic_map_plugin/visibility_control.h"
#include "owen_common/shapes.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include <rclcpp/subscription.hpp>
#include <yaml-cpp/exceptions.h>
#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h>



namespace semantic_map_plugin{
    using nav2_costmap_2d::FREE_SPACE;
    using nav2_costmap_2d::LETHAL_OBSTACLE;
    using nav2_costmap_2d::NO_INFORMATION;

    class SemanticMap : public nav2_costmap_2d::CostmapLayer {
    public:
        SemanticMap();
        virtual void onInitialize();
        virtual void updateBounds(double /*robot_x*/, double /* robot_y*/, double /*robot_yaw*/, double* min_x, double* min_y,
                                  double* max_x, double* maxY);
        virtual void onFootprintChanged();
        virtual void reset();
        virtual void activate();
        virtual void deactivate();
        virtual void updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j);
        virtual bool isClearable() { return false; }

    private:
        void loadConfig();

    private:
        double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
        bool need_recalculation_;
        std::vector<owen_common::cost_rect> no_pass_rects_;
        std::vector<owen_common::cost_rect> pass_rects_;
    };

} // namespace semantic_map_plugin




#endif //NAV_STACK_SEMANTICMAP_HPP
