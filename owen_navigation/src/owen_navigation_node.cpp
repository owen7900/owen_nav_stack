#include <cstdio>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

#include "owen_navigation/NavigatorNode.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Navigation::NavigatorNode>("navigator"));
  rclcpp::shutdown();
}
