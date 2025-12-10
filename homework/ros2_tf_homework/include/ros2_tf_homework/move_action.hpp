#pragma once
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include"homework_interface/action/move.hpp"
#include "ros2_tf_homework/tf_boradcaster_node.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <homework_interface/action/detail/move__struct.hpp>
#include <math.h>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp_action/types.hpp>
#include <thread>

using Move=homework_interface::action::Move;
using GoalHandleMove=rclcpp_action::ServerGoalHandle<Move>;
using namespace std::placeholders;

class MoveActionServer:public rclcpp::Node{
public:
    //构造函数：传入广播器节点指针（用于控制位姿）
    MoveActionServer(std::shared_ptr<TfBroadcasterNode> broadcaster_node):Node("action_node_cpp"),broadcaster_node_(broadcaster_node)
    {
        action_server_=rclcpp_action::create_server<Move>(
            this, 
            "move_action",
            std::bind(&MoveActionServer::handle_goal,this, _1,_2),
            std::bind(&MoveActionServer::handle_cancel, this,_1),
            std::bind(&MoveActionServer::handle_accepted, this,_1)
        );
    }
private:
    //动作服务器实例：
    rclcpp_action::Server<Move>::SharedPtr action_server_;
    //广播器节点指针（用于更新速度和获取位姿）
    std::shared_ptr<TfBroadcasterNode> broadcaster_node_;

    //目标处理:接受或拒绝目标
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Move::Goal> goal
    )
    {
        (void) uuid;
        if (goal->target_x<=10&&goal->target_x>=-10&&goal->target_y<=10&&goal->target_y>=-10) 
        {
            RCLCPP_INFO(this->get_logger(),"提交目标值合法");
        }
        else 
        {
            RCLCPP_INFO(this->get_logger(),"提交的目标x,y必须在-10~10之间");
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    //取消处理：响应取消请求
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMove> goal_handle
    )
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(),"接受的任务取消请求");
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void execute(std::shared_ptr<GoalHandleMove> goal_handle_)
    {
        auto goal=goal_handle_->get_goal();
        auto feedback=std::make_shared<Move::Feedback>();
        rclcpp::Rate rate(10.0);
        while (rclcpp::ok()) 
        {
            //从广播器获取当前位姿
            double current_x=broadcaster_node_->get_x();
            double current_y=broadcaster_node_->get_y();
            double current_yaw=broadcaster_node_->get_yaw();
            //计算剩余距离和目标偏航角
            double remaining_dist=calculate_distance(current_x, current_y, goal->target_x, goal->target_y);
            double target_yaw=calculate_target_yaw(current_x, current_y, goal->target_x, goal->target_y);
            //比例控制计算速度（Kp为比例系数，需调试）
            double angle_error = target_yaw - current_yaw;
            angle_error = std::atan2(std::sin(angle_error), std::cos(angle_error)); 
            double angular_vel=1.5*angle_error;
            double linear_vel=0.5*remaining_dist;
            //feedback数据设置
            feedback->current_x=current_x;
            feedback->current_y=current_y;
            feedback->current_yaw=current_yaw;
            feedback->remaining_distance=remaining_dist;
            //判断是否达到终止条件（0.05）
            if (remaining_dist<=0.05) 
            {
                RCLCPP_INFO(this->get_logger(),"达到目标距离，在误差范围以内");
                broadcaster_node_->set_velocity(0.0, 0.0);
                return;
            }
            //速度限制
            linear_vel=std::min(linear_vel, goal->max_linear_vel);
            angular_vel=std::clamp(angular_vel, -goal->max_angular_vel, goal->max_angular_vel);
            //设置广播器速度
            broadcaster_node_->set_velocity(linear_vel, angular_vel);
            goal_handle_->publish_feedback(feedback);
            //取消处理
            if (goal_handle_->is_canceling()) 
            {
                RCLCPP_INFO(this->get_logger(),"取消结果为:final_x=%lf,final_y=%lf,final_yaw=%lf",
                        current_x,current_y,current_yaw);
                broadcaster_node_->set_velocity(0.0, 0.0);
                 //返回成功结果
                auto result = std::make_shared<Move::Result>();
                result->success = true;
                result->message = "成功到达目标";
                result->final_x = current_x;
                result->final_y = current_y;
                result->final_yaw = current_yaw;
                goal_handle_->succeed(result);
                return;
            }
            rate.sleep();
        }
    }
    //接受目标：启动执行线程
    void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle)
    {
        std::thread(std::bind(&MoveActionServer::execute, this,goal_handle)).detach();
    }

    //辅助函数：计算两点间距离
    double calculate_distance(double x1,double y1,double x2,double y2)
    {
        double dx=x2-x1;
        double dy=y2-y1;
        return std::sqrt(dx*dx+dy*dy);
    }
    //辅助函数：计算目标偏航角（朝向目标点）
    double calculate_target_yaw(double x_current,double y_current,double x_target,double y_target)
    {
        double target_yaw=std::atan2(x_target-x_current,y_target-y_current);
        return target_yaw;
    }
};