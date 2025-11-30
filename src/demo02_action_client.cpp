#include "rclcpp/rclcpp.hpp"
#include <base_interfaces_demo/action/detail/progress__struct.hpp>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/client_goal_handle.hpp>
#include <rclcpp_action/create_client.hpp>
using base_interfaces_demo::action::Progress;
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class ProgressActionClient : public rclcpp::Node {
public:
    ProgressActionClient() : Node("progress_action_client_node_cpp") {
        RCLCPP_INFO(this->get_logger(), "action客户端创建");
        client_ = rclcpp_action::create_client<Progress>(this, "get_sum");
    }

    void send_goal(int num) {
        if (!client_->wait_for_action_server(10s)) {
            RCLCPP_INFO(this->get_logger(), "服务连接失败");
            return;
        }

        auto goal = Progress::Goal();
        goal.num = num;
        
        rclcpp_action::Client<Progress>::SendGoalOptions options;
        options.goal_response_callback = std::bind(&ProgressActionClient::goal_response_callback, this, _1);
        options.feedback_callback = std::bind(&ProgressActionClient::feedback_callback, this, _1, _2);
   
        options.result_callback = std::bind(&ProgressActionClient::result_callback, this, _1);
        
        auto future = client_->async_send_goal(goal, options);
    }

    void goal_response_callback(rclcpp_action::ClientGoalHandle<Progress>::SharedPtr goal_handle) {
        if (!goal_handle) {
            RCLCPP_INFO(this->get_logger(), "目标请求被服务端拒绝");
        } else {
            RCLCPP_INFO(this->get_logger(), "目标处理中");
        }
    }

    void feedback_callback(rclcpp_action::ClientGoalHandle<Progress>::SharedPtr goal_handler, 
                          const std::shared_ptr<const Progress::Feedback> feedback) {
        (void)goal_handler;
        double progress = feedback->progress;
        RCLCPP_INFO(this->get_logger(), "当前进度: %.2f", progress);
    }

    void result_callback(const rclcpp_action::ClientGoalHandle<Progress>::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "最终结果: %ld", result.result->sum);  // 修复：%ld 对应 long
        } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
            RCLCPP_INFO(this->get_logger(), "被中断");
        } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
            RCLCPP_INFO(this->get_logger(), "被取消");
        } else {
            RCLCPP_INFO(this->get_logger(), "未知异常");
        }
    }

private:
    rclcpp_action::Client<Progress>::SharedPtr client_;
};

int main(int argc, char** argv) {
    if (argc != 2) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "请提交一个整形数据");
        return 1;
    }
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ProgressActionClient>();
    node->send_goal(atoi(argv[1]));
    rclcpp::spin(node);  
    rclcpp::shutdown();
    return 0;
}