//
// Created by kevin on 11/8/22.
//

#ifndef MAP_FEATURES_FEATURESINTERFACE_HPP
#define MAP_FEATURES_FEATURESINTERFACE_HPP

#include "rclcpp/rclcpp.hpp"
#include "roomba_msgs/srv/get_path_obstacles.hpp"
#include "roomba_msgs/srv/get_available_destinations.hpp"
#include "owen_common/shapes.hpp"



class FeaturesInterface : public rclcpp::Node {
public:
    explicit FeaturesInterface(const std::string& name);

private:
    void getPathFeatures(const std::shared_ptr<roomba_msgs::srv::GetPathObstacles::Request> request, std::shared_ptr<roomba_msgs::srv::GetPathObstacles::Response> response);
    void getAvailableDestination(const std::shared_ptr<roomba_msgs::srv::GetAvailableDestinations::Request> request, std::shared_ptr<roomba_msgs::srv::GetAvailableDestinations::Response> response);
    void destinationNameCallback(std_msgs::msg::String msg);
    void loadConfig();
    bool isPointInPath(double x, double y, double x1, double x2, double ya, double y2);

private:
    rclcpp::Service<roomba_msgs::srv::GetPathObstacles>::SharedPtr pathFeaturesService_;
    rclcpp::Service<roomba_msgs::srv::GetAvailableDestinations>::SharedPtr availableDestinationsService_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr destinationNameSub_;
    rclcpp::Publisher<roomba_msgs::msg::MultifloorPoint>::SharedPtr destinationPub_;
    std::vector<roomba_msgs::msg::MultifloorRectangle> features_;
    std::vector<roomba_msgs::msg::MultifloorPoint> destinations_;
};


#endif //MAP_FEATURES_FEATURESINTERFACE_HPP
