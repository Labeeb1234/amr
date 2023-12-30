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
#include <vector>

class WheelTf: public rclcpp::Node
{


private:
rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_msg_sub_;
std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

double roll_whl1 = 0.0;
double roll_whl2 = 0.0;
double roll_whl3 = 0.0;
double roll_whl4 = 0.0;

double pitch_whl1 = 0.0;
double pitch_whl2 = 0.0;
double pitch_whl3 = 0.0;
double pitch_whl4 = 0.0;

double yaw_whl1 = 0.0;
double yaw_whl2 = 0.0;
double yaw_whl3 = 0.0;
double yaw_whl4 = 0.0;


public:

WheelTf(): Node("wheel_tf"){
    joint_msg_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", rclcpp::SystemDefaultsQoS(), std::bind(&WheelTf::jointStatesTf, this, std::placeholders::_1));
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}


void jointStatesTf(const sensor_msgs::msg::JointState::SharedPtr joint_msg){

    // char wheel_joint_names[4] = {'Revolute 1', 'Revolute 2', 'Revolute 3', 'Revolute 4'};

    this->pitch_whl1 = joint_msg->position.at(0);
    this->pitch_whl2 = joint_msg->position.at(1);
    this->pitch_whl3 = joint_msg->position.at(2);
    this->pitch_whl4 = joint_msg->position.at(3);


    tf2::Quaternion quaternion1, quaternion2, quaternion3, quaternion4;
    quaternion1.setRPY(this->roll_whl1, this->pitch_whl1, this->yaw_whl1);
    quaternion2.setRPY(this->roll_whl2, this->pitch_whl2, this->yaw_whl2);
    quaternion3.setRPY(this->roll_whl3, this->pitch_whl3, this->yaw_whl3);
    quaternion4.setRPY(this->roll_whl4, this->pitch_whl4, this->yaw_whl4);

    RCLCPP_INFO(this->get_logger(), "wheel_angPosition: %f", this->pitch_whl1);
    RCLCPP_INFO(this->get_logger(), "wheel_angPosition2: %f", this->pitch_whl2);
    RCLCPP_INFO(this->get_logger(), "wheel_angPosition: %f", this->pitch_whl3);
    RCLCPP_INFO(this->get_logger(), "wheel_angPosition: %f", this->pitch_whl4);

    // 0.244627 -->x
    // 0.278967 --> y
    // 0.032980 --> z

    // wheel-1 broad casting fl
    geometry_msgs::msg::TransformStamped wheel_tfb;
    wheel_tfb.header.stamp = this->get_clock()->now();
    wheel_tfb.header.frame_id = "base_link";
    wheel_tfb.child_frame_id = "wheel_hub_1";
    wheel_tfb.transform.translation.x = 0.24467;
    wheel_tfb.transform.translation.y = 0.278967;
    wheel_tfb.transform.translation.z = 0.032980;
    wheel_tfb.transform.rotation.x = quaternion1.x();
    wheel_tfb.transform.rotation.y = quaternion1.y();
    wheel_tfb.transform.rotation.z = quaternion1.z();
    wheel_tfb.transform.rotation.w = quaternion1.w();

    tf_broadcaster_->sendTransform(wheel_tfb);

    // x--> 0.244627
    // y--> -0.279033
    // z--> 0.032980

    // wheel-2 broadcasting fr
    geometry_msgs::msg::TransformStamped wheel_tffr;
    wheel_tffr.header.stamp = this->get_clock()->now();
    wheel_tffr.header.frame_id = "base_link";
    wheel_tffr.child_frame_id = "wheel_hub_2_1";
    wheel_tffr.transform.translation.x = 0.244627;
    wheel_tffr.transform.translation.y = -0.279033;
    wheel_tffr.transform.translation.z = 0.032980;
    wheel_tffr.transform.rotation.x = quaternion2.x();
    wheel_tffr.transform.rotation.y = quaternion2.y();
    wheel_tffr.transform.rotation.z = quaternion2.z();
    wheel_tffr.transform.rotation.w = quaternion2.w();

    tf_broadcaster_->sendTransform(wheel_tffr);
    
    // x--> -0.268017
    // y--> 0.278467
    // z--> 0.032980

    // wheel-3 broadcasting fr
    geometry_msgs::msg::TransformStamped wheel_tfrl;
    wheel_tfrl.header.stamp = this->get_clock()->now();
    wheel_tfrl.header.frame_id = "base_link";
    wheel_tfrl.child_frame_id = "wheel_hub_3_1";
    wheel_tfrl.transform.translation.x = -0.268017;
    wheel_tfrl.transform.translation.y = 0.278467;
    wheel_tfrl.transform.translation.z = 0.032980;
    wheel_tfrl.transform.rotation.x = quaternion3.x();
    wheel_tfrl.transform.rotation.y = quaternion3.y();
    wheel_tfrl.transform.rotation.z = quaternion3.z();
    wheel_tfrl.transform.rotation.w = quaternion3.w();

    tf_broadcaster_->sendTransform(wheel_tfrl);

    // x--> -0.268017
    // y--> -0.278467
    // z--> 0.032980

    // wheel-4 broadcasting fr
    geometry_msgs::msg::TransformStamped wheel_tfrr;
    wheel_tfrr.header.stamp = this->get_clock()->now();
    wheel_tfrr.header.frame_id = "base_link";
    wheel_tfrr.child_frame_id = "wheel_hub_4_1";
    wheel_tfrr.transform.translation.x = -0.268017;
    wheel_tfrr.transform.translation.y = -0.278467;
    wheel_tfrr.transform.translation.z = 0.032980;
    wheel_tfrr.transform.rotation.x = quaternion4.x();
    wheel_tfrr.transform.rotation.y = quaternion4.y();
    wheel_tfrr.transform.rotation.z = quaternion4.z();
    wheel_tfrr.transform.rotation.w = quaternion4.w();

    tf_broadcaster_->sendTransform(wheel_tfrr);


}
};

int main(int argc, char** argv){
    
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<WheelTf>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    
    return 0;
}