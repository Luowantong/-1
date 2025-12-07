#include "ros2_tf_homework/tf_boradcaster_node.hpp"
#include "ros2_tf_homework/tf_listener_node.hpp"
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node1 = std::make_shared<TFListenerNode>();
    auto node2 = std::make_shared<TfBroadcasterNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node1);
    executor.add_node(node2);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}

