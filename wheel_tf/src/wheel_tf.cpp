#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "sensor_msgs/msg/joint_state.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>
#include <cmath>




class WheelTf: public rclcpp::Node
{


private:
rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_msg_sub_;
std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

double roll = 0.0;
double pitch = 0.0;
double yaw = 0.0;

public:

WheelTf(): Node("wheel_tf"){
    joint_msg_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", rclcpp::SystemDefaultsQoS(), std::bind(&WheelTf::jointStatesTf, this, std::placeholders::_1));
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}


void jointStatesTf(const sensor_msgs::msg::JointState::SharedPtr joint_msg){

    std::string wheel_joint_name = "Revolute 1";

    this->pitch = joint_msg->position.at(0);
    tf2::Quaternion quaternion;
    quaternion.setRPY(this->roll, this->pitch, this->yaw);

    RCLCPP_INFO(this->get_logger(), "wheel_angPosition: %f", this->pitch);

    // 0.244627 -->x
    // 0.278967 --> y
    // 0.032980 --> z

    geometry_msgs::msg::TransformStamped wheel_tfb;
    wheel_tfb.header.stamp = this->get_clock()->now();
    wheel_tfb.header.frame_id = "base_link";
    wheel_tfb.child_frame_id = "wheel_hub_1";
    wheel_tfb.transform.translation.x = 0.24467;
    wheel_tfb.transform.translation.y = 0.278967;
    wheel_tfb.transform.translation.z = 0.032980;
    wheel_tfb.transform.rotation.x = quaternion.x();
    wheel_tfb.transform.rotation.y = quaternion.y();
    wheel_tfb.transform.rotation.z = quaternion.z();
    wheel_tfb.transform.rotation.w = quaternion.w();

    tf_broadcaster_->sendTransform(wheel_tfb);

}



};

int main(int argc, char** argv){
    
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<WheelTf>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    
    return 0;
}