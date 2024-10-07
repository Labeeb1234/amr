#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>
#include <cmath>
#include <vector>

using namespace std;

class ImuHandler: public rclcpp::Node{
    private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    float freq=10.0;
    geometry_msgs::msg::Vector3 angular_vel_;
    float roll=0, pitch=0, yaw=0;
    int i=0;


    public:
    ImuHandler(): Node("imu_handler_node"){
        // rclcpp::QoS custom_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu_data", 10, bind(&ImuHandler::get_raw_imu_data, this, placeholders::_1));
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        

        timer_ = this->create_wall_timer(chrono::milliseconds(static_cast<int64_t>(1000/this->freq)), bind(&ImuHandler::process_loop, this));

    };

    ~ImuHandler(){};

    void get_raw_imu_data(const sensor_msgs::msg::Imu::SharedPtr msg){
        this->angular_vel_ = msg->angular_velocity;

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

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, this->yaw);
        q.normalize();

        sensor_msgs::msg::Imu  imu_msg;
        imu_msg.header.stamp = this->get_clock()->now();
        imu_msg.header.frame_id = "imu_link";
        imu_msg.orientation.x = q.x();
        imu_msg.orientation.y = q.y();
        imu_msg.orientation.z = q.z();
        imu_msg.orientation.w = q.w();
        imu_msg.angular_velocity = this->angular_vel_;

        imu_pub_->publish(imu_msg);
    }
};

int main(int argc, char** argv){
    
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ImuHandler>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    
    return 0;
}