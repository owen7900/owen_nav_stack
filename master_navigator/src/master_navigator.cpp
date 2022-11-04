#include "master_navigator/MasterNavigator.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MasterNavigator>("master_navigator"));
  return 0;
}
