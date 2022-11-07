//
// Created by kevin on 11/2/22.
//

#include "../include/semantic_costmap_plugin/semantic_costmap_plugin.hpp"

namespace semantic_costmap_plugin{
    SemanticMap::SemanticMap() : last_min_x_(-std::numeric_limits<float>::max()),
                                 last_min_y_(-std::numeric_limits<float>::max()),
                                 last_max_x_(std::numeric_limits<float>::max()),
                                 last_max_y_(std::numeric_limits<float>::max()){}

    void SemanticMap::onInitialize()
    {
        loadConfig();
        need_recalculation_ = false;
        current_ = true;
    }

    void SemanticMap::loadConfig() {
        auto node = node_.lock();
        if (!node) {
            throw std::runtime_error{"Failed to lock node"};
        }

        declareParameter("features", rclcpp::ParameterValue(std::vector<std::string>{}));
        declareParameter("enabled", rclcpp::ParameterValue(true));
        node->get_parameter(name_ + ".features", features_);
        node->get_parameter(name_ + ".enabled", enabled_);

        for(const auto& val : features_){
            double x_min, x_max, y_min, y_max, cost;
            double zero = 0.0;
            std::cout << val << std::endl;
            declareParameter(val + ".xmin", rclcpp::ParameterValue(zero));
            declareParameter(val + ".xmax", rclcpp::ParameterValue(zero));
            declareParameter(val + ".ymin", rclcpp::ParameterValue(zero));
            declareParameter(val + ".ymax", rclcpp::ParameterValue(zero));
            declareParameter(val + ".cost", rclcpp::ParameterValue(zero));

            node->get_parameter(name_ + "." + val + ".xmin", x_min);
            node->get_parameter(name_ + "." + val + ".xmax", x_max);
            node->get_parameter(name_ + "." + val + ".ymin", y_min);
            node->get_parameter(name_ + "." + val + ".ymax", y_max);
            node->get_parameter(name_ + "." + val + ".cost", cost);

            owen_common::CostRect rect {/*.xmin=*/ x_min, /*.xmax=*/ x_max, /*.ymin=*/ y_min, /*.ymax=*/ y_max, /*.cost=*/ cost};
            no_pass_rects_.push_back(rect);
        }
    }


    void SemanticMap::updateBounds(double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x, double * min_y, double * max_x, double * max_y) {
        if (need_recalculation_) {
            last_min_x_ = *min_x;
            last_min_y_ = *min_y;
            last_max_x_ = *max_x;
            last_max_y_ = *max_y;
            // For some reason when I make these -<double>::max() it does not
            // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
            // -<float>::max() instead.
            *min_x = -std::numeric_limits<float>::max();
            *min_y = -std::numeric_limits<float>::max();
            *max_x = std::numeric_limits<float>::max();
            *max_y = std::numeric_limits<float>::max();
            need_recalculation_ = false;
        } else {
            double tmp_min_x = last_min_x_;
            double tmp_min_y = last_min_y_;
            double tmp_max_x = last_max_x_;
            double tmp_max_y = last_max_y_;
            last_min_x_ = *min_x;
            last_min_y_ = *min_y;
            last_max_x_ = *max_x;
            last_max_y_ = *max_y;
            *min_x = std::min(tmp_min_x, *min_x);
            *min_y = std::min(tmp_min_y, *min_y);
            *max_x = std::max(tmp_max_x, *max_x);
            *max_y = std::max(tmp_max_y, *max_y);
        }
    }


    void SemanticMap::updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j) {
        if (!enabled_) {
            return;
        }

        unsigned char * master_array = master_grid.getCharMap();
        unsigned int size_x = master_grid.getSizeInCellsX();
        unsigned int size_y = master_grid.getSizeInCellsY();

        // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
        // These variables are used to update the costmap only within this window
        // avoiding the updates of whole area.
        // Fixing window coordinates with map size if necessary.
        min_i = std::max(0, min_i);
        min_j = std::max(0, min_j);
        max_i = std::min(static_cast<int>(size_x), max_i);
        max_j = std::min(static_cast<int>(size_y), max_j);

        for(auto i = pass_rects_.begin(); i != pass_rects_.end(); i++){
            auto rect = *i;
            unsigned int xm_min, xm_max, ym_min, ym_max;
            if(master_grid.worldToMap(rect.xmin, rect.ymin, xm_min, ym_min) &&
               master_grid.worldToMap(rect.xmax, rect.xmax, xm_max, ym_max)){
                for(int x = xm_min; x < xm_max; x++){
                    for(int y = ym_min; y < ym_max; y++){
                        if(x < max_i && y < max_j){
                            int index = master_grid.getIndex(x, y);
                            master_array[index] = rect.cost;
                        }
                    }
                }
            }
        }

        for(auto i = no_pass_rects_.begin(); i != no_pass_rects_.end(); i++){
            auto rect = *i;
            unsigned int xm_min, xm_max, ym_min, ym_max;
            if(master_grid.worldToMap(rect.xmin, rect.ymin, xm_min, ym_min) &&
               master_grid.worldToMap(rect.xmax, rect.xmax, xm_max, ym_max)){
                for(int x = xm_min; x < xm_max; x++){
                    for(int y = ym_min; y < ym_max; y++){
                        if(x < max_i && y < max_j){
                            int index = master_grid.getIndex(x, y);
                            master_array[index] = rect.cost;
                        }
                    }
                }
            }
        }
    }


    void SemanticMap::onFootprintChanged()
    {
        need_recalculation_ = true;

        RCLCPP_DEBUG(rclcpp::get_logger(
                "nav2_costmap_2d"), "SemanticMap::onFootprintChanged(): num footprint points: %lu",
                     layered_costmap_->getFootprint().size());
    }

} // namespace semantic_costmap_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(semantic_costmap_plugin::SemanticMap, nav2_costmap_2d::Layer)