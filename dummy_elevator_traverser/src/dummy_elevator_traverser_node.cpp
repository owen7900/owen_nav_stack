#include "dummy_elevator_traverser/DummyElevatorTraverser.hpp"
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyElevatorTraverser>("elevator_traverser"));
  rclcpp::shutdown();
  return 0;
}
