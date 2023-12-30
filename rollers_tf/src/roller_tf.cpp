#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>
#include <cmath>
#include <vector>

class RollerTF: public rclcpp::Node
{
private:
std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
rclcpp::TimerBase::SharedPtr timer_;

public:

RollerTF(): Node("roller_tf"){
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&RollerTF::RollerTFCallback, this));

}

void RollerTFCallback(){
    // wheel-1
    geometry_msgs::msg::TransformStamped rtf1_msg;
    rtf1_msg.header.stamp = this->get_clock()->now();
    rtf1_msg.header.frame_id = "wheel_hub_1";
    rtf1_msg.child_frame_id = "roller_1_1";
    rtf1_msg.transform.translation.x = 0.230188;
    rtf1_msg.transform.translation.y = 0.323283;
    rtf1_msg.transform.translation.z = 0.071812;
    geometry_msgs::msg::TransformStamped rtf2_msg;
    rtf2_msg.header.stamp = this->get_clock()->now();
    rtf2_msg.header.frame_id = "wheel_hub_1";
    rtf2_msg.child_frame_id = "roller_1_2";
    rtf2_msg.transform.translation.x = 0.261847;
    rtf2_msg.transform.translation.y = 0.323283;
    rtf2_msg.transform.translation.z = 0.070662;
    geometry_msgs::msg::TransformStamped rtf3_msg;
    rtf3_msg.header.stamp = this->get_clock()->now();
    rtf3_msg.header.frame_id = "wheel_hub_1";
    rtf3_msg.child_frame_id = "roller_1_3";
    rtf3_msg.transform.translation.x = 0.283465;
    rtf3_msg.transform.translation.y = 0.323283;
    rtf3_msg.transform.translation.z = 0.047403;
    geometry_msgs::msg::TransformStamped rtf4_msg;
    rtf4_msg.header.stamp = this->get_clock()->now();
    rtf4_msg.header.frame_id = "wheel_hub_1";
    rtf4_msg.child_frame_id = "roller_1_4";
    rtf4_msg.transform.translation.x = 0.282284;
    rtf4_msg.transform.translation.y = 0.323283;
    rtf4_msg.transform.translation.z = 0.015714;
    geometry_msgs::msg::TransformStamped rtf5_msg;
    rtf5_msg.header.stamp = this->get_clock()->now();
    rtf5_msg.header.frame_id = "wheel_hub_1";
    rtf5_msg.child_frame_id = "roller_1_5";
    rtf5_msg.transform.translation.x = 0.259036;
    rtf5_msg.transform.translation.y = 0.323283;
    rtf5_msg.transform.translation.z = -0.005711;
    geometry_msgs::msg::TransformStamped rtf6_msg;
    rtf6_msg.header.stamp = this->get_clock()->now();
    rtf6_msg.header.frame_id = "wheel_hub_1";
    rtf6_msg.child_frame_id = "roller_1_6";
    rtf6_msg.transform.translation.x = 0.227381;
    rtf6_msg.transform.translation.y = 0.323283;
    rtf6_msg.transform.translation.z = -0.004685;
    geometry_msgs::msg::TransformStamped rtf7_msg;
    rtf7_msg.header.stamp = this->get_clock()->now();
    rtf7_msg.header.frame_id = "wheel_hub_1";
    rtf7_msg.child_frame_id = "roller_1_7";
    rtf7_msg.transform.translation.x = 0.205786;
    rtf7_msg.transform.translation.y = 0.323283;
    rtf7_msg.transform.translation.z = 0.018573;
    geometry_msgs::msg::TransformStamped rtf8_msg;
    rtf8_msg.header.stamp = this->get_clock()->now();
    rtf8_msg.header.frame_id = "wheel_hub_1";
    rtf8_msg.child_frame_id = "roller_1_8";
    rtf8_msg.transform.translation.x = 0.206962;
    rtf8_msg.transform.translation.y = 0.323283;
    rtf8_msg.transform.translation.z = 0.050240;

    // wheel-2
    geometry_msgs::msg::TransformStamped rtf9_msg;
    rtf9_msg.header.stamp = this->get_clock()->now();
    rtf9_msg.header.frame_id = "wheel_hub_2_1";
    rtf9_msg.child_frame_id = "roller_2_1";
    rtf9_msg.transform.translation.x = 0.261551;
    rtf9_msg.transform.translation.y = -0.323351;
    rtf9_msg.transform.translation.z = 0.070788;
    geometry_msgs::msg::TransformStamped rtf10_msg;
    rtf10_msg.header.stamp = this->get_clock()->now();
    rtf10_msg.header.frame_id = "wheel_hub_2_1";
    rtf10_msg.child_frame_id = "roller_2_2";
    rtf10_msg.transform.translation.x = 0.229801;
    rtf10_msg.transform.translation.y = -0.323351;
    rtf10_msg.transform.translation.z = 0.071659;
    geometry_msgs::msg::TransformStamped rtf11_msg;
    rtf11_msg.header.stamp = this->get_clock()->now();
    rtf11_msg.header.frame_id = "wheel_hub_2_1";
    rtf11_msg.child_frame_id = "roller_2_3";
    rtf11_msg.transform.translation.x = 0.283360;
    rtf11_msg.transform.translation.y = -0.323351;
    rtf11_msg.transform.translation.z = 0.047674;
    geometry_msgs::msg::TransformStamped rtf12_msg;
    rtf12_msg.header.stamp = this->get_clock()->now();
    rtf12_msg.header.frame_id = "wheel_hub_2_1";
    rtf12_msg.child_frame_id = "roller_2_4";
    rtf12_msg.transform.translation.x = 0.282416;
    rtf12_msg.transform.translation.y = -0.323351;
    rtf12_msg.transform.translation.z = 0.015994;
    geometry_msgs::msg::TransformStamped rtf13_msg;
    rtf13_msg.header.stamp = this->get_clock()->now();
    rtf13_msg.header.frame_id = "wheel_hub_2_1";
    rtf13_msg.child_frame_id = "roller_2_5";
    rtf13_msg.transform.translation.x = 0.259441;
    rtf13_msg.transform.translation.y = -0.323351;
    rtf13_msg.transform.translation.z = -0.005718;
    geometry_msgs::msg::TransformStamped rtf14_msg;
    rtf14_msg.header.stamp = this->get_clock()->now();
    rtf14_msg.header.frame_id = "wheel_hub_2_1";
    rtf14_msg.child_frame_id = "roller_2_6";
    rtf14_msg.transform.translation.x = 0.227782;
    rtf14_msg.transform.translation.y = -0.323351;
    rtf14_msg.transform.translation.z = -0.004874;
    geometry_msgs::msg::TransformStamped rtf15_msg;
    rtf15_msg.header.stamp = this->get_clock()->now();
    rtf15_msg.header.frame_id = "wheel_hub_2_1";
    rtf15_msg.child_frame_id = "roller_2_7";
    rtf15_msg.transform.translation.x = 0.205880;
    rtf15_msg.transform.translation.y = -0.323351;
    rtf15_msg.transform.translation.z = 0.018312;
    geometry_msgs::msg::TransformStamped rtf16_msg;
    rtf16_msg.header.stamp = this->get_clock()->now();
    rtf16_msg.header.frame_id = "wheel_hub_2_1";
    rtf16_msg.child_frame_id = "roller_2_8";
    rtf16_msg.transform.translation.x = 0.206823;
    rtf16_msg.transform.translation.y = -0.323351;
    rtf16_msg.transform.translation.z = 0.049922;

    // wheel-3
    geometry_msgs::msg::TransformStamped rtf17_msg;
    rtf17_msg.header.stamp = this->get_clock()->now();
    rtf17_msg.header.frame_id = "wheel_hub_3_1";
    rtf17_msg.child_frame_id = "roller_3_1";
    rtf17_msg.transform.translation.x = -0.253535;
    rtf17_msg.transform.translation.y = 0.322784;
    rtf17_msg.transform.translation.z = 0.071784;
    geometry_msgs::msg::TransformStamped rtf18_msg;
    rtf18_msg.header.stamp = this->get_clock()->now();
    rtf18_msg.header.frame_id = "wheel_hub_3_1";
    rtf18_msg.child_frame_id = "roller_3_2";
    rtf18_msg.transform.translation.x = -0.285410;
    rtf18_msg.transform.translation.y = 0.322784;
    rtf18_msg.transform.translation.z = 0.070570;
    geometry_msgs::msg::TransformStamped rtf19_msg;
    rtf19_msg.header.stamp = this->get_clock()->now();
    rtf19_msg.header.frame_id = "wheel_hub_3_1";
    rtf19_msg.child_frame_id = "roller_3_3";
    rtf19_msg.transform.translation.x = -0.306884;
    rtf19_msg.transform.translation.y = 0.322784;
    rtf19_msg.transform.translation.z = 0.047312;
    geometry_msgs::msg::TransformStamped rtf20_msg;
    rtf20_msg.header.stamp = this->get_clock()->now();
    rtf20_msg.header.frame_id = "wheel_hub_3_1";
    rtf20_msg.child_frame_id = "roller_3_4";
    rtf20_msg.transform.translation.x = -0.305597;
    rtf20_msg.transform.translation.y = 0.322784;
    rtf20_msg.transform.translation.z = 0.015532;
    geometry_msgs::msg::TransformStamped rtf21_msg;
    rtf21_msg.header.stamp = this->get_clock()->now();
    rtf21_msg.header.frame_id = "wheel_hub_3_1";
    rtf21_msg.child_frame_id = "roller_3_5";
    rtf21_msg.transform.translation.x = -0.282335;
    rtf21_msg.transform.translation.y = 0.322784;
    rtf21_msg.transform.translation.z = -0.005908;
    geometry_msgs::msg::TransformStamped rtf22_msg;
    rtf22_msg.header.stamp = this->get_clock()->now();
    rtf22_msg.header.frame_id = "wheel_hub_3_1";
    rtf22_msg.child_frame_id = "roller_3_6";
    rtf22_msg.transform.translation.x = -0.250654;
    rtf22_msg.transform.translation.y = 0.322784;
    rtf22_msg.transform.translation.z = -0.004643;
    geometry_msgs::msg::TransformStamped rtf23_msg;
    rtf23_msg.header.stamp = this->get_clock()->now();
    rtf23_msg.header.frame_id = "wheel_hub_3_1";
    rtf23_msg.child_frame_id = "roller_3_7";
    rtf23_msg.transform.translation.x = -0.230388;
    rtf23_msg.transform.translation.y = 0.322784;
    rtf23_msg.transform.translation.z = 0.050303;
    geometry_msgs::msg::TransformStamped rtf24_msg;
    rtf24_msg.header.stamp = this->get_clock()->now();
    rtf24_msg.header.frame_id = "wheel_hub_3_1";
    rtf24_msg.child_frame_id = "roller_3_8";
    rtf24_msg.transform.translation.x = -0.229148;
    rtf24_msg.transform.translation.y = 0.322784;
    rtf24_msg.transform.translation.z = 0.018637;

    // wheel-4
    geometry_msgs::msg::TransformStamped rtf25_msg;
    rtf25_msg.header.stamp = this->get_clock()->now();
    rtf25_msg.header.frame_id = "wheel_hub_4_1";
    rtf25_msg.child_frame_id = "roller_4_1";
    rtf25_msg.transform.translation.x = -0.258407;
    rtf25_msg.transform.translation.y = -0.323845;
    rtf25_msg.transform.translation.z = 0.073289;
    geometry_msgs::msg::TransformStamped rtf26_msg;
    rtf26_msg.header.stamp = this->get_clock()->now();
    rtf26_msg.header.frame_id = "wheel_hub_4_1";
    rtf26_msg.child_frame_id = "roller_4_2";
    rtf26_msg.transform.translation.x = -0.289705;
    rtf26_msg.transform.translation.y = -0.323845;
    rtf26_msg.transform.translation.z = 0.068289;
    geometry_msgs::msg::TransformStamped rtf27_msg;
    rtf27_msg.header.stamp = this->get_clock()->now();
    rtf27_msg.header.frame_id = "wheel_hub_4_1";
    rtf27_msg.child_frame_id = "roller_4_3";
    rtf27_msg.transform.translation.x = -0.308326;
    rtf27_msg.transform.translation.y = -0.323845;
    rtf27_msg.transform.translation.z = 0.042548;
    geometry_msgs::msg::TransformStamped rtf28_msg;
    rtf28_msg.header.stamp = this->get_clock()->now();
    rtf28_msg.header.frame_id = "wheel_hub_4_1";
    rtf28_msg.child_frame_id = "roller_4_4";
    rtf28_msg.transform.translation.x = -0.303293;
    rtf28_msg.transform.translation.y = -0.323845;
    rtf28_msg.transform.translation.z = 0.011266;
    geometry_msgs::msg::TransformStamped rtf29_msg;
    rtf29_msg.header.stamp = this->get_clock()->now();
    rtf29_msg.header.frame_id = "wheel_hub_4_1";
    rtf29_msg.child_frame_id = "roller_4_5";
    rtf29_msg.transform.translation.x = -0.277553;
    rtf29_msg.transform.translation.y = -0.323845;
    rtf29_msg.transform.translation.z = -0.007107;
    geometry_msgs::msg::TransformStamped rtf30_msg;
    rtf30_msg.header.stamp = this->get_clock()->now();
    rtf30_msg.header.frame_id = "wheel_hub_4_1";
    rtf30_msg.child_frame_id = "roller_4_6";
    rtf30_msg.transform.translation.x = -0.246292;
    rtf30_msg.transform.translation.y = -0.323845;
    rtf30_msg.transform.translation.z = -0.002300;
    geometry_msgs::msg::TransformStamped rtf31_msg;
    rtf31_msg.header.stamp = this->get_clock()->now();
    rtf31_msg.header.frame_id = "wheel_hub_4_1";
    rtf31_msg.child_frame_id = "roller_4_7";
    rtf31_msg.transform.translation.x = -0.227705;
    rtf31_msg.transform.translation.y = -0.323845;
    rtf31_msg.transform.translation.z = 0.023396;
    geometry_msgs::msg::TransformStamped rtf32_msg;
    rtf32_msg.header.stamp = this->get_clock()->now();
    rtf32_msg.header.frame_id = "wheel_hub_4_1";
    rtf32_msg.child_frame_id = "roller_4_8";
    rtf32_msg.transform.translation.x = -0.232726;
    rtf32_msg.transform.translation.y = -0.323845;
    rtf32_msg.transform.translation.z = 0.054677;


    tf_broadcaster_->sendTransform(rtf1_msg);
    tf_broadcaster_->sendTransform(rtf2_msg);
    tf_broadcaster_->sendTransform(rtf3_msg);
    tf_broadcaster_->sendTransform(rtf4_msg);
    tf_broadcaster_->sendTransform(rtf5_msg);
    tf_broadcaster_->sendTransform(rtf6_msg);
    tf_broadcaster_->sendTransform(rtf7_msg);
    tf_broadcaster_->sendTransform(rtf8_msg);
    tf_broadcaster_->sendTransform(rtf9_msg);
    tf_broadcaster_->sendTransform(rtf10_msg);
    tf_broadcaster_->sendTransform(rtf11_msg);
    tf_broadcaster_->sendTransform(rtf12_msg);
    tf_broadcaster_->sendTransform(rtf13_msg);
    tf_broadcaster_->sendTransform(rtf14_msg);
    tf_broadcaster_->sendTransform(rtf15_msg);
    tf_broadcaster_->sendTransform(rtf16_msg);
    tf_broadcaster_->sendTransform(rtf17_msg);
    tf_broadcaster_->sendTransform(rtf18_msg);
    tf_broadcaster_->sendTransform(rtf19_msg);
    tf_broadcaster_->sendTransform(rtf20_msg);
    tf_broadcaster_->sendTransform(rtf21_msg);
    tf_broadcaster_->sendTransform(rtf22_msg);
    tf_broadcaster_->sendTransform(rtf23_msg);
    tf_broadcaster_->sendTransform(rtf24_msg);
    tf_broadcaster_->sendTransform(rtf25_msg);
    tf_broadcaster_->sendTransform(rtf26_msg);
    tf_broadcaster_->sendTransform(rtf27_msg);
    tf_broadcaster_->sendTransform(rtf28_msg);
    tf_broadcaster_->sendTransform(rtf29_msg);
    tf_broadcaster_->sendTransform(rtf30_msg);
    tf_broadcaster_->sendTransform(rtf31_msg);
    tf_broadcaster_->sendTransform(rtf32_msg);

}

};

int main(int argc, char** argv){
    
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<RollerTF>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    
    return 0;
}