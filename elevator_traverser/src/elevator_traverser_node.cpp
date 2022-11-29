#include "elevator_traverser/ElevatorTraverser.hpp"
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ElevatorTraverser>("elevator_traverser"));
  rclcpp::shutdown();
  return 0;
}
