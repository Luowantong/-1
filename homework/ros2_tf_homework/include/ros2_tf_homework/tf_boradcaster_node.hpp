#pragma once
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "homework_interface/msg/velocity.hpp"
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <homework_interface/msg/detail/velocity__struct.hpp>
#include <math.h>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>
#include <std_srvs/srv/detail/trigger__struct.hpp>
#include <string>
#include <tf2_ros/transform_broadcaster.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "std_srvs/srv/trigger.hpp"
#include <cmath>
#include "geometry_msgs/msg/quaternion.hpp"
#include <tf2/LinearMath/Quaternion.h>

class TfBroadcasterNode:public rclcpp::Node{
public:
    TfBroadcasterNode():Node("tf_broadcaster_node_cpp"){
        RCLCPP_INFO(this->get_logger(), "动态TF广播器节点启动!");
        tf_broadcaster_=std::make_shared<tf2_ros::TransformBroadcaster>(this);
        vel_sub_ = this->create_subscription<homework_interface::msg::Velocity>(
            "/velocity_cmd", 
            10,
            std::bind(&TfBroadcasterNode::velocity_callback, this, std::placeholders::_1)
        );
        reset_service_=this->create_service<std_srvs::srv::Trigger>(
            "/reset_pose",
            std::bind(&TfBroadcasterNode::handler_reset,this,std::placeholders::_1,std::placeholders::_2));
        this->declare_parameter("initial_x", 0.0);
        this->declare_parameter("initial_y", 0.0);
        this->declare_parameter("initial_yaw", 0.0);
        this->declare_parameter("parent_frame","base_link");
        this->declare_parameter("child_frame", "child_link");
        this->declare_parameter("publish_freq", 10.0);

        this->get_parameter("initial_x", x_);
        this->get_parameter("initial_y", y_);
        this->get_parameter("initial_yaw", yaw_);
        this->get_parameter("parent_frame", parent_frame_);
        this->get_parameter("child_frame", child_frame_);
        this->get_parameter("publish_freq", publish_freq_);

        linear_vel_ = 0.0;
        angular_vel_ = 0.0;  // 统一使用带下划线的版本
        last_time_ = this->now();
        int timer_period_ms = static_cast<int>(1000.0 / publish_freq_);
        timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_period_ms),
        std::bind(&TfBroadcasterNode::timer_callback, this)
        );
    }
private:
    //速度话题回调函数
    void velocity_callback(const homework_interface::msg::Velocity::SharedPtr msg)
    {
        linear_vel_=msg->linear_vel;
        angular_vel_=msg->angular_vel;
    }
    //定时器回调函数
    void timer_callback()
    {
        auto current_time_=this->now();
        double dt=(current_time_-last_time_).seconds();
        last_time_=current_time_;
        x_+=linear_vel_*cos(yaw_)*dt;
        y_+=linear_vel_*sin(yaw_)*dt;
        yaw_+=angular_vel_*dt;
        yaw_=atan2(sin(yaw_), cos(yaw_));
        tf_publish();
    }
    //重置位姿函数（Service调用）
    void reset_pose(double x,double y,double yaw)
    {
        x_=x;
        y_=y;
        yaw_=yaw;
        linear_vel_=0.0;
        angular_vel_=0.0;
    }
    void tf_publish()
    {
        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp=this->get_clock()->now();
        ts.header.frame_id=parent_frame_;
        ts.child_frame_id=child_frame_;
        ts.transform.translation.x=x_;
        ts.transform.translation.y=y_;
        ts.transform.translation.z=0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw_);
        ts.transform.rotation.x=q.x();
        ts.transform.rotation.y=q.y();
        ts.transform.rotation.z=q.z();
        ts.transform.rotation.w=q.w();

        tf_broadcaster_->sendTransform(ts);
    }
    void handler_reset(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        double init_x,init_y,init_yaw;
        this->get_parameter("initial_x",init_x);
        this->get_parameter("initial_y",init_y);
        this->get_parameter("initial_yaw",init_yaw);
        reset_pose(init_x, init_y, init_yaw);

        response->success=true;
        response->message="位姿已重置: x=" + std::to_string(x_) + 
                    ", y=" + std::to_string(y_) + 
                    ", yaw=" + std::to_string(yaw_);
        RCLCPP_INFO(this->get_logger(),"服务调用，位姿已重置");
        tf_publish();
    }
    //订阅速度话题
    rclcpp::Subscription<homework_interface::msg::Velocity>::SharedPtr vel_sub_;
    //控制TF发布频率
    rclcpp::TimerBase::SharedPtr timer_;
    //TF广播器实例
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    //重置位姿服务实例
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    //位姿参数
    double x_;
    double y_;
    double yaw_;
    //速度参数
    double linear_vel_;
    double angular_vel_;
    //时间戳——记录上一次更新时间
    rclcpp::Time last_time_;
    //配置参数
    std::string parent_frame_;
    std::string child_frame_;
    double publish_freq_;
};