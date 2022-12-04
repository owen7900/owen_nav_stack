//
// Created by kevin on 11/30/22.
//

#ifndef CAPSTON_WS_MATH_HPP
#define CAPSTON_WS_MATH_HPP

#include "roomba_msgs/msg/multifloor_rectangle.hpp"

namespace owen_common::math{
    bool isPointInRect(const roomba_msgs::msg::MultifloorRectangle &rect, double x, double y);
    double Angle2D(double x1, double y1, double x2, double y2);
}


#endif //CAPSTON_WS_MATH_HPP
