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
        std::string label;

        CostRect(double x_min, double x_max, double y_min, double y_max, double rect_cost, std::string floor_num, std::string l) : xmin(x_min), xmax(x_max), ymin(y_min), ymax(y_max), cost(rect_cost), floor(floor_num), label(l){}
    };

    struct LabeledMultiFloorPoint{
        double x;
        double y;
        std::string label;
        std::string floor;

        LabeledMultiFloorPoint(double xval, double yval, std::string f, std::string l) : x(xval), y(yval), label(l), floor(f){}
    };
}


#endif //SEMANTIC_MAP_PLUGIN_SHAPES_HPP
