#include"rclcpp/rclcpp.hpp"
#include <atomic>
#include <base_interfaces_demo/srv/detail/add_ints__builder.hpp>
#include <base_interfaces_demo/srv/detail/add_ints__struct.hpp>
#include <functional>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include"base_interfaces_demo/srv/add_ints.hpp"
using base_interfaces_demo::srv::AddInts;
using std::make_shared;
using std::placeholders::_1;
using std::placeholders::_2;

class AddIntsServer : public rclcpp::Node {
public:
  AddIntsServer():Node("add_ints_server_node_cpp"){
    RCLCPP_INFO(this->get_logger(),"服务端节点创建!");
    server_=this->create_service<AddInts>("add_ints", std::bind(&AddIntsServer::add,this,_1,_2));
  }
private:
  void add(const AddInts::Request::SharedPtr req,const AddInts::Response::SharedPtr res){
    res->sum=req->num1+req->num2;
    RCLCPP_INFO(this->get_logger(),"%d+%d=%d",req->num1,req->num2,res->sum);
  }
  rclcpp::Service<AddInts>::SharedPtr server_;
};

int main(int argc,char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<AddIntsServer>());
  rclcpp::shutdown();
  return 0;
}

