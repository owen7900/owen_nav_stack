//
// Created by kevin on 11/8/22.
//

#include "../include/map_features/FeaturesInterface.hpp"
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Dense>

namespace CONSTANTS{
    const std::string ConfigPath = "/home/kevin/capston_ws/src/nav_stack/owen_bringup/config/multi_floor_params.yaml";
}


using Line2 = Eigen::Hyperplane<double,2>;
using Vec2  = Eigen::Vector2d;

FeaturesInterface::FeaturesInterface(const std::string& name) : rclcpp::Node(name){
    loadConfig();

    destinationPub_ = this->create_publisher<roomba_msgs::msg::MultifloorPoint>("/destination", 10);
    destinationNameSub_ = this->create_subscription<std_msgs::msg::String>(
            "/destinationName", 10, std::bind(&FeaturesInterface::destinationNameCallback, this, std::placeholders::_1));

    pathFeaturesService_ = this->create_service<roomba_msgs::srv::GetPathObstacles>(
            "path_features", std::bind(&FeaturesInterface::getPathFeatures, this, std::placeholders::_1, std::placeholders::_2)
            );

    availableDestinationsService_ = this->create_service<roomba_msgs::srv::GetAvailableDestinations>(
            "available_destinations", std::bind(&FeaturesInterface::getAvailableDestination, this, std::placeholders::_1, std::placeholders::_2)
    );
}

void FeaturesInterface::loadConfig() {
    RCLCPP_INFO_STREAM(this->get_logger(), "Parsing " << CONSTANTS::ConfigPath);
    YAML::Node top = YAML::LoadFile(CONSTANTS::ConfigPath);
    RCLCPP_INFO_STREAM(this->get_logger(), "Parsed " << CONSTANTS::ConfigPath);

    auto features = top["features"];
    for(const auto &feature : features){
        auto vals = feature.second;

        auto xmin = vals["xmin"].as<double>();
        auto xmax = vals["xmax"].as<double>();
        auto ymin = vals["ymin"].as<double>();
        auto ymax = vals["ymax"].as<double>();
        auto floor = vals["floor"].as<std::string>();
        auto label = vals["label"].as<std::string>();

        auto label_msg = std_msgs::msg::String();
        label_msg.data = label;

        auto floor_msg = std_msgs::msg::String();
        floor_msg.data = floor;

        auto point1_msg = geometry_msgs::msg::Point();
        point1_msg.x = xmin;
        point1_msg.y = ymin;

        auto point2_msg = geometry_msgs::msg::Point();
        point2_msg.x = xmax;
        point2_msg.y = ymax;

        auto feature_msg = roomba_msgs::msg::MultifloorRectangle();
        feature_msg.floor_id = floor_msg;
        feature_msg.p1 = point1_msg;
        feature_msg.p2 = point2_msg;
        feature_msg.label = label_msg;

        features_.push_back(feature_msg);
    }

    auto destinations = top["destinations"];
    for(const auto &destination : destinations){
        auto vals = destination.second;

        double x = vals["x"].as<double>();
        double y = vals["y"].as<double>();
        std::string floor = vals["floor"].as<std::string>();
        std::string label = vals["label"].as<std::string>();

        auto label_msg = std_msgs::msg::String();
        label_msg.data = label;

        auto floor_msg = std_msgs::msg::String();
        floor_msg.data = floor;

        auto point_msg = geometry_msgs::msg::Point();
        point_msg.x = x;
        point_msg.y = y;

        auto dest_msg = roomba_msgs::msg::MultifloorPoint();
        dest_msg.point = point_msg;
        dest_msg.floor_id = floor_msg;
        dest_msg.label = label_msg;

        destinations_.push_back(dest_msg);
    }
}

void FeaturesInterface::getPathFeatures(const std::shared_ptr<roomba_msgs::srv::GetPathObstacles::Request> request,
                                        std::shared_ptr<roomba_msgs::srv::GetPathObstacles::Response> response) {
    std::vector<roomba_msgs::msg::MultifloorRectangle> pathFeatures;
    roomba_msgs::msg::MultifloorPoint lastPoint;
    int i = 0;
    for (const auto &point : request->path.points){
        if(i == 0){
            i++;
            lastPoint = point;
            continue;
        }

        Vec2 pathPoint1(lastPoint.point.x, lastPoint.point.y);
        Vec2 pathPoint2(point.point.x, point.point.y);
        Line2 path_line = Line2::Through(pathPoint1, pathPoint2);

        for(const auto& rect : features_){
            if(point.floor_id.data == rect.floor_id.data){
                // | left
                Vec2 p1(rect.p1.x, rect.p1.y);
                Vec2 p2(rect.p1.x, rect.p2.y);
                Line2 left = Line2::Through(p1, p2);
                auto intersection = path_line.intersection(left);
                if(isPointInPath(intersection.x(), intersection.y(), pathPoint1.x(), pathPoint2.x(), pathPoint1.y(), pathPoint2.y())){
                        pathFeatures.push_back(rect);
                        continue;
                }

                // -- top
                Vec2 p3(rect.p1.x, rect.p2.y);
                Vec2 p4(rect.p2.x, rect.p2.y);
                Line2 top = Line2::Through(p3, p4);
                intersection = path_line.intersection(top);
                if(isPointInPath(intersection.x(), intersection.y(), pathPoint1.x(), pathPoint2.x(), pathPoint1.y(), pathPoint2.y())){
                        pathFeatures.push_back(rect);
                        continue;
                }

                // | right
                Vec2 p5(rect.p2.x, rect.p1.y);
                Vec2 p6(rect.p2.x, rect.p2.y);
                Line2 right = Line2::Through(p5, p6);
                intersection = path_line.intersection(right);
                if(isPointInPath(intersection.x(), intersection.y(), pathPoint1.x(), pathPoint2.x(), pathPoint1.y(), pathPoint2.y())){
                        pathFeatures.push_back(rect);
                        continue;
                }

                // -- bottom
                Vec2 p7(rect.p1.x, rect.p1.y);
                Vec2 p8(rect.p2.x, rect.p1.y);
                Line2 bottom = Line2::Through(p7, p8);
                intersection = path_line.intersection(right);
                if(isPointInPath(intersection.x(), intersection.y(), pathPoint1.x(), pathPoint2.x(), pathPoint1.y(), pathPoint2.y())){
                        pathFeatures.push_back(rect);
                        continue;
                }
            }
        }
    }

    response->features_in_path = pathFeatures;
}

void FeaturesInterface::getAvailableDestination(
        const std::shared_ptr<roomba_msgs::srv::GetAvailableDestinations::Request> request,
        std::shared_ptr<roomba_msgs::srv::GetAvailableDestinations::Response> response) {
    response->destinations = destinations_;
}

void FeaturesInterface::destinationNameCallback(std_msgs::msg::String msg) {
    for(const auto &destination : destinations_){
        if(destination.label.data == msg.data){
            destinationPub_->publish(destination);
        }
    }
}


bool FeaturesInterface::isPointInPath(double x, double y, double x1, double x2, double y1, double y2) {
    if(x >= x1 &&
       x <= x2 &&
       y >= y1 &&
       y <= y2){
        return true;
    }
    return false;
}

