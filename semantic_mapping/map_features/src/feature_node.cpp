#include "../include/map_features/FeaturesInterface.hpp"
#include <rclcpp/executors/multi_threaded_executor.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exe;
    std::shared_ptr<FeaturesInterface> featuresNode = std::make_shared<FeaturesInterface>("map_features");

    exe.add_node(featuresNode->get_node_base_interface());
    exe.spin();

    return 0;
}