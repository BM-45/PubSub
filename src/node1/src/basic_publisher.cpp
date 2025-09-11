#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
// rosidl cmake package compiles your .msg file and generates c++ header file.
#include "node1/msg/basic_message.hpp"

using namespace std::chrono_literals;

class MyNodePublisher : public rclcpp::Node{
// class variables
rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Publisher<node1::msg::BasicMessage>::SharedPtr publisher_;

// Here you are calling the base class contructor before this  constructor is called.
public: MyNodePublisher(): Node("practice"){

// creating a publisher to the communication channel topic. 10 number represents the queue size.
publisher_ = this->create_publisher<node1::msg::BasicMessage>("topic", 10);

// this create_wall_timer creates a timer_base instance, after every 500ms
// it pushes timer_callback callback into the event loop in the spin function.
timer_ = this->create_wall_timer(500ms, std::bind(&MyNodePublisher::timer_callback, this));
}

// triggers when event occurs.
private: void timer_callback(){
// Your message
auto message = node1::msg::BasicMessage();
message.text = "hi bhanu";
message.value = 9.9;
message.array = {1, 2};

// There are different log levels in ros, we are usig INFO log level here.
RCLCPP_INFO(this->get_logger(), "Publishing: name=%s, number=%.2f, readdings=%zu", 
	    message.text.c_str(), message.value, message.array.size());

// publish the message
publisher_->publish(message);
}
};



int main(int argc,char **argv){

//  command line expression is passed and ros2 arguments are quoted,
//  and process are started for the creating nodes, publishers and subscribers
//  sets up communication infrastructure
rclcpp::init(argc, argv);

//  Creates MyNode class object  in heap and shared pointer address is given back.
//  spin function is used to manage the lifecycle of the node.
//  internally it checks for the loops , call corresponding callbacks.
//  Note You shouldnt create MyNode in stack.
rclcpp::spin(std::make_shared<MyNodePublisher>());

// DDs middle ware to disconnect
// stops the background threads used by rclcpp.
rclcpp::shutdown();

return 0;

}
 

