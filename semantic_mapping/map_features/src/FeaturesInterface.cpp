//
// Created by kevin on 11/8/22.
//

#include "../include/map_features/FeaturesInterface.hpp"
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Dense>
#include "owen_common/math.hpp"

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

        auto x1 = vals["x1"].as<double>();
        auto y1 = vals["y1"].as<double>();
        auto x2 = vals["x2"].as<double>();
        auto y2 = vals["y2"].as<double>();
        auto x3 = vals["x3"].as<double>();
        auto y3 = vals["y3"].as<double>();
        auto x4 = vals["x4"].as<double>();
        auto y4 = vals["y4"].as<double>();
        auto floor = vals["floor"].as<std::string>();
        auto label = vals["label"].as<std::string>();

        roomba_msgs::msg::MultifloorRectangle feature_msg;
        feature_msg.p1.x = x1;
        feature_msg.p1.y = y1;
        feature_msg.p2.x = x2;
        feature_msg.p2.y = y2;
        feature_msg.p3.x = x3;
        feature_msg.p3.y = y3;
        feature_msg.p4.x = x4;
        feature_msg.p4.y = y4;
        feature_msg.floor_id.data = floor;
        feature_msg.label.data = label;

        features_.push_back(feature_msg);
    }

    auto destinations = top["destinations"];
    for(const auto &destination : destinations){
        auto vals = destination.second;

        double x = vals["x"].as<double>();
        double y = vals["y"].as<double>();
        std::string floor = vals["floor"].as<std::string>();
        std::string label = vals["label"].as<std::string>();

        roomba_msgs::msg::MultifloorPoint dest_msg;
        dest_msg.point.x = x;
        dest_msg.point.y = y;
        dest_msg.floor_id.data = floor;
        dest_msg.label.data = label;

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

        for(const auto& rect : features_){
            if(point.floor_id.data == rect.floor_id.data){
                if(owen_common::math::isPointInRect(rect, pathPoint1.x(), pathPoint1.y()) &&
                   owen_common::math::isPointInRect(rect, pathPoint2.x(), pathPoint2.y())){
                    pathFeatures.push_back(rect);
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

