#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>
#include <cmath>


class BaseLinkTf: public rclcpp::Node
{

private:

rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_msg_sub_;
std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
rclcpp::TimerBase::SharedPtr timer_;

double x_pose = 0.0;
double y_pose = 0.0;
double th_pose = 0.0;

// global quaternion variables
double quat_x = 0.0;
double quat_y = 0.0;
double quat_z = 0.0;
double quat_w = 0.0;

public:

BaseLinkTf(): Node("base_link_tf"){
    odom_msg_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("amr_ust/odom_pose", rclcpp::SystemDefaultsQoS(), std::bind(&BaseLinkTf::OdomCallback, this, std::placeholders::_1)); // can add optional params inside SystemDefaultQos()
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&BaseLinkTf::TfBroadcaster, this));


}

void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg){
    this->x_pose = odom_msg->pose.pose.position.x;
    this->y_pose = odom_msg->pose.pose.position.y;

    // updating quaternion values
    this->quat_x = odom_msg->pose.pose.orientation.x;
    this->quat_y = odom_msg->pose.pose.orientation.y;
    this->quat_z = odom_msg->pose.pose.orientation.z;
    this->quat_w = odom_msg->pose.pose.orientation.w;

    tf2::Quaternion quat(
        odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z,
        odom_msg->pose.pose.orientation.w
    );

    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    this->th_pose = yaw;
    RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, th: %f", this->x_pose, this->y_pose, this->th_pose);

}

void TfBroadcaster(){

    geometry_msgs::msg::TransformStamped transform_stamped_msg;

    transform_stamped_msg.header.stamp = this->get_clock()->now();
    transform_stamped_msg.header.frame_id = "odom";
    transform_stamped_msg.child_frame_id = "base_link";
    transform_stamped_msg.transform.translation.x = this->x_pose;
    transform_stamped_msg.transform.translation.y = this->y_pose;
    transform_stamped_msg.transform.translation.z = 0.0;
    transform_stamped_msg.transform.rotation.x = this->quat_x;
    transform_stamped_msg.transform.rotation.y = this->quat_y;
    transform_stamped_msg.transform.rotation.z = this->quat_z;
    transform_stamped_msg.transform.rotation.w = this->quat_w;


    tf_broadcaster_->sendTransform(transform_stamped_msg);


}

};


int main(int argc, char** argv){
    
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<BaseLinkTf>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    
    return 0;
}