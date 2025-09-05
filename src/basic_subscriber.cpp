#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "node1/msg/basic_message.hpp"

// placeholders used in bind.
using std::placeholders::_1;


class MyNodeSubscriber: public rclcpp::Node{

//class variables
rclcpp::Subscription<node1::msg::BasicMessage>::SharedPtr subscription_;


// constructor
public: MyNodeSubscriber(): Node("practice_subscriber"){

// subscription object
// executors sends the placeholder message into this subscription method

subscription_ = this->create_subscription<node1::msg::BasicMessage>("topic", 10, std::bind(&MyNodeSubscriber::topic_callback, this, _1));

} 

private: void topic_callback(const node1::msg::BasicMessage::SharedPtr msg) const{

RCLCPP_INFO(this->get_logger(), "I heard: '%s' '%.2f' '%zu'", msg->text.c_str(), msg->value, msg->array.size());

}

};

int main(int argc, char * argv[]){

rclcpp::init(argc,argv);

rclcpp::spin(std::make_shared<MyNodeSubscriber>());

rclcpp::shutdown();

return 0;

}
