#include"rclcpp/rclcpp.hpp"
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/timer.hpp>
#include"base_interfaces_demo/msg/student.hpp"
using base_interfaces_demo::msg::Student;
using namespace std::chrono_literals;
class TalkerStu:public rclcpp::Node{
public:
    TalkerStu():Node("talkerstu_node_cpp"){
        publisher_=this->create_publisher<Student>("chatter_stu",10);
        timer_=this->create_wall_timer(500ms,std::bind(&TalkerStu::on_timer,this));
    }
private:
    void on_timer(){
        auto stu=Student();
        stu.name="huluwa";
        stu.age=8+age;
        stu.height=2.20;
        age++;
        publisher_->publish(stu);
        RCLCPP_INFO(this->get_logger(),"发布的消息:(%s,%d,%.2f)",stu.name.c_str(),stu.age,stu.height);
    }
    rclcpp::Publisher<Student>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int age=0;
};

int main(int argc,char const** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<TalkerStu>());
    rclcpp::shutdown();
    return 0;
}