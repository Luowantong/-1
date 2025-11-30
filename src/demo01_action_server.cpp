#include "rclcpp/rclcpp.hpp"
#include <base_interfaces_demo/action/detail/progress__struct.hpp>
#include <functional>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp_action/types.hpp>
#include <rcutils/error_handling.h>
#include <thread>
#include "rclcpp_action/rclcpp_action.hpp"
#include"base_interfaces_demo/action/progress.hpp"
using base_interfaces_demo::action::Progress;
using std::placeholders::_1;
using std::placeholders::_2;

class ProgressActionServer : public rclcpp::Node {
public:
    ProgressActionServer() : Node("progress_action_server_node_cpp") {
        RCLCPP_INFO(this->get_logger(), "action服务端创建");
        server_ = rclcpp_action::create_server<Progress>(
            this,
            "get_sum",
            std::bind(&ProgressActionServer::handle_goal, this, _1, _2),
            std::bind(&ProgressActionServer::handle_cancel, this, _1),
            std::bind(&ProgressActionServer::handle_accepted, this, _1)
        );
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Progress::Goal> goal) {
        (void)uuid;
        if (goal->num <= 1) {
            RCLCPP_INFO(this->get_logger(), "提交目标值必须大于1");
            return rclcpp_action::GoalResponse::REJECT;
        } else {
            RCLCPP_INFO(this->get_logger(), "提交目标值合法");
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle) {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "接受的任务取消请求");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle) {
        int num = goal_handle->get_goal()->num;
        int sum = 0;
        auto feedback = std::make_shared<Progress::Feedback>();
        rclcpp::Rate rate(1.0);
        auto result = std::make_shared<Progress::Result>();
        
        for (int i = 1; i <= num; i++) {
            sum += i;
            double progress = i / (double)num;
            feedback->progress = progress;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "连续反馈中，进度:%.2f", progress);
            
            if (goal_handle->is_canceling()) {
                result->sum = sum;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "任务被取消，当前结果: %d", sum);
                return;
            }
            rate.sleep();
        }
        
        if (rclcpp::ok()) {
            result->sum = sum;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "最终结果: %d", sum);
        }
    }

    void handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle) {
        std::thread(std::bind(&ProgressActionServer::execute, this, goal_handle)).detach();
    }

private:
    rclcpp_action::Server<Progress>::SharedPtr server_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ProgressActionServer>());
    rclcpp::shutdown();
    return 0;
}