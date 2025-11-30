#include"rclcpp/rclcpp.hpp"
#include <base_interfaces_demo/srv/detail/add_ints__struct.hpp>
#include <cstdlib>
#include <future>
#include <memory>
#include <rclcpp/client.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/future_return_code.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include"base_interfaces_demo/srv/add_ints.hpp"
using base_interfaces_demo::srv::AddInts;
using std::make_shared;
using std::shared_ptr;
using namespace std::chrono_literals;

class AddIntsClient : public rclcpp::Node {
public:
    AddIntsClient() : Node("add_ints_client_node_cpp") {
        RCLCPP_INFO(this->get_logger(), "客户端创建!");
        client_ = this->create_client<AddInts>("add_ints");
    }

    // 连接服务器实现
    bool connect_server() {
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "强行终止客户端");
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "服务连接中");
        }
        return true;
    }

    // 发送请求 - 使用正确的返回类型
    auto send_request(int num1, int num2) {
        // 组织请求数据
        auto request = std::make_shared<AddInts::Request>();
        request->num1 = num1;
        request->num2 = num2;
        return client_->async_send_request(request);
    }

private:
    rclcpp::Client<AddInts>::SharedPtr client_;
};

int main(int argc, char** argv) {
    if (argc != 3) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "请提交两个整形数字");
        return 1;
    }

    rclcpp::init(argc, argv);
    
    // 创建客户端对象
    auto client = std::make_shared<AddIntsClient>();
    
    // 调用客户端对象的连接服务器功能
    bool flag = client->connect_server();
    
    if (!flag) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "服务器连接失败，程序退出");
        return 0;
    }

    // 执行后续操作
    auto future = client->send_request(atoi(argv[1]), atoi(argv[2]));
    
    // 处理响应
    if (rclcpp::spin_until_future_complete(client, future) == rclcpp::FutureReturnCode::SUCCESS) {
        // 获取响应结果
        auto response = future.get();
        RCLCPP_INFO(client->get_logger(), "响应成功 sum= %d", response->sum);
    } else {
        RCLCPP_INFO(client->get_logger(), "响应失败");
    }
    
    rclcpp::shutdown();
    return 0;
}