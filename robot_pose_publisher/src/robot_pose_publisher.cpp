#include <chrono>
#include <functional>
#include <memory>
#include <string>


#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"


class RobotPosePublisher: public rclcpp::Node{

private:
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
std::string to_frame_rel_ = "base_footprint";
std::string from_frame_rel_ = "map";

rclcpp::TimerBase::SharedPtr timer_;

public:

RobotPosePublisher(): Node("robot_pose_publisher"){
    this->pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot_pose", 10);
    this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);

    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&RobotPosePublisher::pose_publisher, this)); // 50ms --> 1000/50 = 20Hz 

}

~RobotPosePublisher(){}


void pose_publisher(){
    // Look up for the transformation between map and odom frames
    geometry_msgs::msg::TransformStamped tr;
    try { 
        
        tr = tf_buffer_->lookupTransform(to_frame_rel_, from_frame_rel_, tf2::TimePointZero);
    } 
    catch (const tf2::TransformException & ex) {
        RCLCPP_INFO( this->get_logger(), "Could not find transform from %s to %s: %s", to_frame_rel_.c_str(), from_frame_rel_.c_str(), ex.what());
        return;
    }
    geometry_msgs::msg::PoseStamped msg;
    // translational tf of the bot
    msg.pose.position.x = tr.transform.translation.x;
    msg.pose.position.y = tr.transform.translation.y;
    msg.pose.position.z = 0.0;
    msg.pose.orientation = tr.transform.rotation;

    // orientation of the bot in roll, pitch and yaw
    // double roll, pitch, yaw;
    // tf2::Quaternion q = tr.transform.rotation;
    // tf2::Matrix3x3 mat(q);
    // mat.getRPY(roll, pitch, yaw);
    // RCLCPP_INFO(this->get_logger(), "[roll: %f, pitch: %f, yaw: %f]", roll, pitch, yaw);

    this->pose_pub_->publish(msg);

}


};



int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotPosePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;

}



