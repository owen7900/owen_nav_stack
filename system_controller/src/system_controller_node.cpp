#include <cstdio>
#include <memory>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include "system_controller/SystemController.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exe;
  std::shared_ptr<SystemController> controllerNode = std::make_shared<SystemController>("system_controller");

  exe.add_node(controllerNode->get_node_base_interface());

  exe.spin();

  return 0;
}
