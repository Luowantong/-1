#include "ros2_tf_homework/tf_boradcaster_node.hpp"
#include "ros2_tf_homework/tf_listener_node.hpp"
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include "ros2_tf_homework/move_action.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto broadcaster = std::make_shared<TfBroadcasterNode>();
    auto listener = std::make_shared<TFListenerNode>();
    auto action_server = std::make_shared<MoveActionServer>(broadcaster);  
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(broadcaster);
    executor.add_node(listener);
    executor.add_node(action_server);  
    
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

