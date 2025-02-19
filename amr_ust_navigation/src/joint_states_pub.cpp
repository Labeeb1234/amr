#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <memory>
#include <functional>
#include <vector>
#include <iostream>

using namespace std;

class JointStatesPublisher: public rclcpp::Node{
private:
rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;

public:
JointStatesPublisher(): Node("joint_states_publisher_node"){
    // --------------------------------------------------------
    rclcpp::QoS custom_qos(rclcpp::KeepLast(10));
    custom_qos.reliability(rclcpp::ReliabilityPolicy::Reliable); // (TCP)data integrity but latency may not be that low
    custom_qos.durability(rclcpp::DurabilityPolicy::Volatile);
    // -------------------------------------------------------- (Just Put it out here)
    joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states",
        rclcpp::SystemDefaultsQoS(),
        bind(&JointStatesPublisher::joint_states_callback, this, placeholders::_1)
    );


}

void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg){

}


};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = make_shared<JointStatesPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}