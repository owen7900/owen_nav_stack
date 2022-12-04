//
// Created by kevin on 11/30/22.
//

#include "../include/owen_common/math.hpp"
#include <eigen3/Eigen/Dense>

using Line2 = Eigen::Hyperplane<double,2>;
using Vec2  = Eigen::Vector2d;

namespace owen_common::math{
    bool isPointInRect(const roomba_msgs::msg::MultifloorRectangle &rect, double x , double y){
        double angle=0;
        double p1x, p1y, p2x, p2y;
        Vec2 point(x, y);
        int n = 4;
        std::vector<Vec2> polygon;
        polygon.push_back(Vec2(rect.p1.x, rect.p1.y));
        polygon.push_back(Vec2(rect.p2.x, rect.p2.y));
        polygon.push_back(Vec2(rect.p3.x, rect.p3.y));
        polygon.push_back(Vec2(rect.p4.x, rect.p4.y));
        for (int i=0;i<n;i++) {
            p1x = polygon[i].x() - point.x();
            p1y = polygon[i].y() - point.y();
            p2x = polygon[(i+1)%n].x() - point.x();
            p2y= polygon[(i+1)%n].y() - point.y();
            angle += Angle2D(p1x, p1y, p2x, p2y);
        }

        if (std::abs(angle) < M_PI)
            return false;
        else
            return true;
    }

    double Angle2D(double x1, double y1, double x2, double y2) {
        double dtheta,theta1,theta2;

        theta1 = atan2(y1,x1);
        theta2 = atan2(y2,x2);
        dtheta = theta2 - theta1;
        while (dtheta > M_PI)
            dtheta -= (2.0*M_PI);
        while (dtheta < (-1.0*M_PI))
            dtheta += (2.0*M_PI);

        return(dtheta);
    }

}