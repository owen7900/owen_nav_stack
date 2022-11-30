//
// Created by kevin on 11/2/22.
//

#include "semantic_costmap_plugin/semantic_costmap_plugin.hpp"

#include <eigen3/Eigen/Dense>
#include "owen_common/math.hpp"

using Line2 = Eigen::Hyperplane<double,2>;
using Vec2  = Eigen::Vector2d;

namespace semantic_costmap_plugin {
    SemanticMap::SemanticMap() : last_min_x_(-std::numeric_limits<float>::max()),
                                 last_min_y_(-std::numeric_limits<float>::max()),
                                 last_max_x_(std::numeric_limits<float>::max()),
                                 last_max_y_(std::numeric_limits<float>::max()) {}

    void SemanticMap::onInitialize() {
        setup();
        need_recalculation_ = false;
        current_ = true;
    }

    void SemanticMap::setup() {
        auto node = node_.lock();
        if (!node) {
            throw std::runtime_error{"Failed to lock node"};
        }
        floor_sub_ = node->create_subscription<std_msgs::msg::String>("/floor_id", 10,
                                                                      std::bind(&SemanticMap::floorCallback, this,
                                                                                std::placeholders::_1));

        declareParameter("no_pass", rclcpp::ParameterValue(std::vector<std::string>{}));
        declareParameter("enabled", rclcpp::ParameterValue(true));
        node->get_parameter(name_ + ".no_pass", no_pass_params_);
        node->get_parameter(name_ + ".enabled", enabled_);

        for (const auto &val: no_pass_params_) {
            double x1, x2, x3, x4, y1, y2, y3, y4;
            std::string floor;
            double zero = 0.0;
            std::cout << val << std::endl;
            declareParameter(val + ".x1", rclcpp::ParameterValue(zero));
            declareParameter(val + ".x2", rclcpp::ParameterValue(zero));
            declareParameter(val + ".x3", rclcpp::ParameterValue(zero));
            declareParameter(val + ".x4", rclcpp::ParameterValue(zero));
            declareParameter(val + ".y1", rclcpp::ParameterValue(zero));
            declareParameter(val + ".y2", rclcpp::ParameterValue(zero));
            declareParameter(val + ".y3", rclcpp::ParameterValue(zero));
            declareParameter(val + ".y4", rclcpp::ParameterValue(zero));
            declareParameter(val + ".floor", rclcpp::ParameterValue(""));

            node->get_parameter(name_ + "." + val + ".x1", x1);
            node->get_parameter(name_ + "." + val + ".x2", x2);
            node->get_parameter(name_ + "." + val + ".x3", x3);
            node->get_parameter(name_ + "." + val + ".x4", x4);
            node->get_parameter(name_ + "." + val + ".y1", y1);
            node->get_parameter(name_ + "." + val + ".y2", y2);
            node->get_parameter(name_ + "." + val + ".y3", y3);
            node->get_parameter(name_ + "." + val + ".y4", y4);
            node->get_parameter(name_ + "." + val + ".floor", floor);

            roomba_msgs::msg::MultifloorRectangle rect;
            rect.p1.x = x1;
            rect.p1.y = y1;
            rect.p2.x = x2;
            rect.p2.y = y2;
            rect.p3.x = x3;
            rect.p3.y = y3;
            rect.p4.x = x4;
            rect.p4.y = y4;

            no_pass_rects_.push_back(rect);
        }
    }

    void SemanticMap::floorCallback(const std_msgs::msg::String::SharedPtr msg) {
        floor_ = msg->data;
    }


    void SemanticMap::updateBounds(double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double *min_x,
                                   double *min_y, double *max_x, double *max_y) {
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


    void SemanticMap::updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) {
        if (!enabled_) {
            return;
        }

        unsigned char *master_array = master_grid.getCharMap();
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

        for (auto i = no_pass_rects_.begin(); i != no_pass_rects_.end(); i++) {
            auto rect = *i;
            if (rect.floor_id.data == floor_) {
                unsigned int x1, x2, x3, x4, y1, y2, y3, y4;
                if (master_grid.worldToMap(rect.p1.x, rect.p1.y, x1, y1) &&
                    master_grid.worldToMap(rect.p2.x, rect.p2.y, x2, y2) &&
                    master_grid.worldToMap(rect.p3.x, rect.p3.y, x3, y3) &&
                    master_grid.worldToMap(rect.p4.x, rect.p4.y, x4, y4)) {
                    roomba_msgs::msg::MultifloorRectangle rectMapPoints;
                    rectMapPoints.p1.x = x1;
                    rectMapPoints.p1.y = y1;
                    rectMapPoints.p2.x = x2;
                    rectMapPoints.p2.y = y2;
                    rectMapPoints.p3.x = x3;
                    rectMapPoints.p3.y = y3;
                    rectMapPoints.p4.x = x4;
                    rectMapPoints.p4.y = y4;
                    for (int x = min_i; x < max_i; x++) {
                        for (int y = min_j; y < max_j; y++) {
                            if(owen_common::math::isPointInRect(rectMapPoints, x, y)){
                                int index = master_grid.getIndex(x, y);
                                master_array[index] = 254.0;
                            }
                        }
                    }
                }

            }
        }
    }


    void SemanticMap::onFootprintChanged() {
        need_recalculation_ = true;

        RCLCPP_DEBUG(rclcpp::get_logger(
                "nav2_costmap_2d"), "SemanticMap::onFootprintChanged(): num footprint points: %lu",
                     layered_costmap_->getFootprint().size());
    }


} // namespace semantic_costmap_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(semantic_costmap_plugin::SemanticMap, nav2_costmap_2d::Layer)