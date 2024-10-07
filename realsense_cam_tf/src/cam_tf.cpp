#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>
#include <cmath>
#include <vector>

using namespace std;

class RealsenseImu: public rclcpp::Node{
    private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr aligned_depth_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    

    float freq=20.0;
    geometry_msgs::msg::Vector3 angular_vel_;
    geometry_msgs::msg::Vector3 linear_accel_;
    int16_t depth_value_;
    float roll=0, pitch=0, yaw=0;
    int i=0;

    public:
    RealsenseImu(): Node("realsense_imu_node"){
        rclcpp::QoS custom_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
        custom_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort); 

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("camera/camera/imu", custom_qos, bind(&RealsenseImu::cam_gyro_callback, this, placeholders::_1));
        aligned_depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>("camera/camera/aligned_depth_to_color/image_raw", 10, bind(&RealsenseImu::depth_data_callback, this, placeholders::_1));
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("camera/camera/yaw", 10);
        

        timer_ = this->create_wall_timer(chrono::milliseconds(static_cast<int64_t>(1000/this->freq)), bind(&RealsenseImu::process_loop, this));

    };

    ~RealsenseImu(){};

    void depth_data_callback(const sensor_msgs::msg::Image::SharedPtr msg){
        if (msg->data.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty depth image.");
            return;
        }
        // Image dimensions
        int width = msg->width;
        int height = msg->height;

        // Calculate the center pixel (cX, cY)
        int cX = width / 2;
        int cY = height / 2;

        // Assuming depth image format is 16UC1 (16-bit unsigned integer, single channel)
        // Each depth value is stored as a 16-bit unsigned integer
        int index = (cY * width + cX) * 2;  // Multiply by 2 because each pixel is 2 bytes (16 bits)

        // Convert the two bytes back to a 16-bit unsigned int (depth in [mm])
        this->depth_value_ = msg->data[index] | (msg->data[index + 1] << 8);
    }

    void cam_gyro_callback(const sensor_msgs::msg::Imu::SharedPtr msg){
        this->angular_vel_ = msg->angular_velocity;
        this->linear_accel_ = msg->linear_acceleration;
    }

    void process_loop(){
        float dt = 1/this->freq;
        while(i<1){
            RCLCPP_INFO(this->get_logger(), "Processing loop executed");
            i++;
        }
        // RCLCPP_INFO(this->get_logger(), "DEPTH VALUE: %d", this->depth_value_);
        this->roll += this->angular_vel_.x*dt;
        this->pitch += this->angular_vel_.y*dt;
        this->yaw += this->angular_vel_.z*dt;

        // Accelerometer-based angle estimation
        float accel_roll, accel_pitch;
        accel_to_angle(linear_accel_, accel_roll, accel_pitch);

        // Complementary filter to fuse angles
        float alpha = 0.75; // Complementary filter coefficient
        this->roll = alpha * this->roll + (1 - alpha) * accel_roll;
        this->pitch = alpha * this->pitch + (1 - alpha) * accel_pitch;

        tf2::Quaternion q;
        q.setRPY(-1.57, 0.0, 1.57-this->yaw);
        q.normalize();

        sensor_msgs::msg::Imu  imu_msg;
        imu_msg.orientation.x = q.x();
        imu_msg.orientation.y = q.y();
        imu_msg.orientation.z = q.z();
        imu_msg.orientation.w = q.w();
        imu_msg.angular_velocity = this->angular_vel_;

        imu_pub_->publish(imu_msg);

        // Example of broadcasting the transform using tf2
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->get_clock()->now();
        transformStamped.header.frame_id = "base_link"; // Change as necessary
        transformStamped.child_frame_id = "camera_link";   // Change as necessary
        transformStamped.transform.translation.x = 0.331142;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.054538;
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transformStamped);

    }

    void accel_to_angle(const geometry_msgs::msg::Vector3& linear_accel, float& roll, float& pitch) {
        // Calculate roll (rotation around X-axis)
        roll = atan2(linear_accel.y, linear_accel.z);
        // Calculate pitch (rotation around Y-axis)
        pitch = atan2(-linear_accel.x, sqrt(linear_accel.y * linear_accel.y + linear_accel.z * linear_accel.z));
        // all angles in radians
    }

};

int main(int argc, char** argv){
    
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<RealsenseImu>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    
    return 0;
}