#pragma once
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <string>
using namespace std::chrono_literals;
class TFListenerNode:public rclcpp::Node{
public:
    TFListenerNode():Node("tf_listener_node_cpp"){
        this->declare_parameter("parent_frame", "base_link");
        this->declare_parameter("child_frame", "target_link");
        this->declare_parameter("query_freq", 10.0);
        this->get_parameter("parent_frame",parent_frame);
        this->get_parameter("child_frame",child_frame);
        this->get_parameter("query_freq",query_freq_);

        tf_buffer_=std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_=std::make_shared<tf2_ros::TransformListener>(*tf_buffer_,this);
        int timer_period_ms = static_cast<int>(1000.0 / query_freq_);
        timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_period_ms), 
        std::bind(&TFListenerNode::timer_callback, this)
        );
    }
private:
    //定时器回调函数：定时查询TF变换
    void timer_callback()
    {
        try 
        {
            auto ts=tf_buffer_->lookupTransform(parent_frame, child_frame, tf2::TimePointZero);
            RCLCPP_INFO(this->get_logger(),"---------转换完成的坐标帧信息----------");
            RCLCPP_INFO(this->get_logger(),
                    "新坐标帧：父坐标系：%s,子坐标系：%s,偏移量(%.2f,%.2f,%.2f)",
                ts.header.frame_id.c_str(),
                ts.child_frame_id.c_str(),
                ts.transform.translation.x,
                ts.transform.translation.y,
                ts.transform.translation.z
            );
            RCLCPP_INFO(this->get_logger(), "旋转: qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f", 
                ts.transform.rotation.x,
                ts.transform.rotation.y,
                ts.transform.rotation.z,
                ts.transform.rotation.w);
        } 
        catch (const tf2::LookupException& e) 
        {
            RCLCPP_INFO(this->get_logger(),"异常提示:%s",e.what());
        }

    }
    //TF缓冲区：存储接收的TF变换(默认缓存10s)
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    //TF监听器：监听TF广播并写入缓冲区
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    //定时器：控制查询频率
    rclcpp::TimerBase::SharedPtr timer_;
    //配置参数：目标父坐标系，子坐标系，查询频率
    std::string parent_frame;
    std::string child_frame;
    double query_freq_;
};