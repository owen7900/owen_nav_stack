//
// Created by kevin on 11/1/22.
//

#ifndef SEMANTIC_MAP_PLUGIN_SHAPES_HPP
#define SEMANTIC_MAP_PLUGIN_SHAPES_HPP

#include <string>

namespace owen_common{
    struct CostRect{
        double xmin;
        double xmax;
        double ymin;
        double ymax;
        double cost;
        std::string floor;

        CostRect(double x_min, double x_max, double y_min, double y_max, double rect_cost, std::string floor_num) : xmin(x_min), xmax(x_max), ymin(y_min), ymax(y_max), cost(rect_cost), floor(floor_num){}
    };
}


#endif //SEMANTIC_MAP_PLUGIN_SHAPES_HPP
