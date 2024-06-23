#include "gazebo/common/Time.hh"
#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"
#include "gazebo_ros/conversions/builtin_interfaces.hpp"
#include "gazebo_ros/conversions/geometry_msgs.hpp"
#include "gazebo_ros/node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/callback_group.hpp"

#include "geometry_msgs/msg/wrench.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int8.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/vector3.hpp"


#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/transform_listener.h"

#include <memory>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <thread>
#include <stdio.h>

#include "my_plugin/my_plugin.hpp"


using namespace std;

namespace gazebo_plugins
{

class MyPluginPrivate{

public:

    MyPluginPrivate(){};
    ~MyPluginPrivate(){};

    void InitSubscribers(string wrench_cmd_topic);
    void InitPublishers(string odom_pose_topic);
    void Reset();

    // ros2 node instance
    rclcpp::Node::SharedPtr ros_node_{nullptr};
    // gazebo_ros::Node::SharedPtr ros_node_{nullptr};

    std::mutex lock_;
    gazebo::event::ConnectionPtr update_connection_;
    
    gazebo::physics::WorldPtr world; // world link pointer
    vector<gazebo::physics::LinkPtr> link; // vector of link pointer
    vector<gazebo::physics::JointPtr> joint; 
    gazebo::physics::ModelPtr model; // model pointer

    gazebo::common::Time current_time;
    gazebo::common::Time last_time;

    double last_odom_publish_time{0.0};

    string cmd_topic;
    string odom_topic;
    double odom_hz = 30.0;
    int num_links_;

    double fx, fy, fz; // or use ignition::math::Vector3<double>& force;
    // State of the system/model/bot
    ignition::math::Pose3<double> pose;
    ignition::math::Vector3<double> euler;
    ignition::math::Vector3<double> velocity, acceleration, angular_velocity, position;
    
    void OnOdomPublish(
        const gazebo::common::Time &current_sim_time,
        const ignition::math::Pose3<double> & pose,
        const ignition::math::Vector3<double> & velocity,
        const ignition::math::Vector3<double> & acceleration
    );

private:
    // rclcpp::SubscriptionOptions create_subscription_options(string topic_name, uint32_t queue_size);

    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr wrench_msg_sub_{nullptr};
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_msg_pub_{nullptr};
    rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};
    shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{nullptr};

    void OnForceCmd(const geometry_msgs::msg::Wrench::SharedPtr force_msg); 



};

MyPlugin::MyPlugin(): impl_(make_unique<MyPluginPrivate>()){}
MyPlugin::~MyPlugin(){impl_->update_connection_.reset();}


void MyPlugin::Load(const gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf){
    impl_->ros_node_ = gazebo_ros::Node::Get(_sdf, "");

    if(!rclcpp::ok()){
    RCLCPP_ERROR(impl_->ros_node_->get_logger(),
      "A ROS node for Gazebo has not been initialized, unable to load plugin. Load the Gazebo system plugin 'libgazebo_ros_init.so' in the gazebo_ros package"); 
    }

    impl_->model = _model;
    impl_->world = _model->GetWorld();

    RCLCPP_INFO(impl_->ros_node_->get_logger(), "My Plugin is loading....");


    // getting number of links to control
    impl_->num_links_ = _sdf->Get<int>("num_links", 1).first;
    vector<gazebo::physics::LinkPtr> links;
    // need to get links
    auto base_footprint_elem = _sdf->GetElement("base_footprint");
    auto base_footprint_name = base_footprint_elem->Get<string>();
    auto base_footprint_link = _model->GetLink(base_footprint_name);
    RCLCPP_INFO(impl_->ros_node_->get_logger(), "link_name: %s", base_footprint_name.c_str());
    links.push_back(base_footprint_link);

    for(auto link_elem=_sdf->GetElement("link"); link_elem!=nullptr; link_elem=link_elem->GetNextElement("link")){
        auto link_name = link_elem->Get<string>();
        auto link = _model->GetLink(link_name);
        RCLCPP_INFO(impl_->ros_node_->get_logger(), "link_name: %s", link_name.c_str());
        links.push_back(link);

    }
    for(int i=0; i<impl_->num_links_; i++){
        impl_->link.push_back(links[i]);
    }

    // get cmd topic
    if(!_sdf->HasElement("cmd_topic_name")){
        RCLCPP_ERROR(impl_->ros_node_->get_logger(), "no command topic name specified!");
    }
    else{
        impl_->cmd_topic = _sdf->GetElement("cmd_topic_name")->Get<string>();
    }

    // get odom topic
    if(!_sdf->HasElement("odom_topic_name")){
        RCLCPP_ERROR(impl_->ros_node_->get_logger(), "no odom topic name specified!");
    }
    else{
        impl_->odom_topic = _sdf->GetElement("odom_topic_name")->Get<string>();
        if(!_sdf->HasElement("odom_hz")){
            impl_->odom_hz = 30.0;
            RCLCPP_INFO(impl_->ros_node_->get_logger(), "No odom publisher rate specified setting to default publisher rate value: [%lf]", impl_->odom_hz);
        }
        else{
            impl_->odom_hz = _sdf->GetElement("odom_hz")->Get<double>();
            RCLCPP_INFO(impl_->ros_node_->get_logger(), "Setting odom publisher rate: [%lf]", impl_->odom_hz);
        }
    }

    // initiate publisher and subscribers
    impl_->InitSubscribers(impl_->cmd_topic);
    impl_->InitPublishers(impl_->odom_topic);


    impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&MyPlugin::Update, this,  std::placeholders::_1)
    );

    RCLCPP_INFO(impl_->ros_node_->get_logger(), "My Plugin finished loading...!");
    impl_->last_time = impl_->world->SimTime();
}

void MyPlugin::Update(const gazebo::common::UpdateInfo &_info){
    impl_->current_time = _info.simTime;
    double dt = (impl_->current_time-impl_->last_time).Double();
    if(dt==0.0){return;}
    // applying a simple force to the link under consideration in the y-direction wrt to base_link
    // getting the base_link(model COM pose/vel/acceleration from gazebo positioning system)
    impl_->pose = impl_->link[0]->WorldPose();
    impl_->euler = impl_->pose.Rot().Euler();
    impl_->acceleration = (impl_->link[0]->WorldLinearVel() - impl_->velocity)/dt;
    impl_->velocity = impl_->link[0]->WorldLinearVel();
    // cout << impl_->pose << endl;

    // visible acutation
    impl_->link[0]->AddRelativeForce(ignition::math::Vector3<double>(0.0, impl_->fy, 0.0));



    // controlling the publish rate of odom data as well as the odom->base_footprint tf broadcaster
    if(impl_->current_time.Double()-impl_->last_odom_publish_time >=(1.0/impl_->odom_hz)){
        impl_->OnOdomPublish(impl_->current_time, impl_->pose, impl_->velocity, impl_->acceleration);
        impl_->last_odom_publish_time = impl_->current_time.Double();
    }
    
    impl_->last_time = impl_->current_time;

}


void MyPluginPrivate::Reset(){
    // restting forces and torques on a body
    for(auto link_elem:link ){
        link_elem->SetForce(ignition::math::Vector3<double>(0.0, 0.0, 0.0));
        link_elem->SetTorque(ignition::math::v6::Vector3<double>(0.0, 0.0, 0.0));
    }

    // link->SetForce(ignition::math::Vector3<double>(0.0, 0.0, 0.0));
    // link->SetTorque(ignition::math::v6::Vector3<double>(0.0, 0.0, 0.0));

    // Reset the state of the system/model/bot
    pose.Reset();
    velocity.Set();
    angular_velocity.Set();
    acceleration.Set();
    euler.Set();

    last_time.Set(0,0);


}

void MyPluginPrivate::InitSubscribers(string wrench_cmd_topic){
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    // auto qos = rclcpp::QoS(1);
    if(!wrench_cmd_topic.empty()){
        
        RCLCPP_INFO(ros_node_->get_logger(), "Subscribing to [%s] topic", wrench_cmd_topic.c_str());

        wrench_msg_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Wrench>(
            wrench_cmd_topic, qos, bind(&MyPluginPrivate::OnForceCmd, this, placeholders::_1)
        );
    }else{
        RCLCPP_ERROR(ros_node_->get_logger(), "No wrench cmd topic defined...!");
    }
}

void MyPluginPrivate::InitPublishers(string odom_pose_topic){
    auto queue_size = 10;
    if(!odom_pose_topic.empty()){
        
        RCLCPP_INFO(ros_node_->get_logger(), "Publishing odom data to [%s] topic", odom_pose_topic.c_str());

        odom_msg_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>(
            odom_pose_topic, queue_size
        );
    }else{
        RCLCPP_ERROR(ros_node_->get_logger(), "No odom pose topic defined...!");
    } 
    
    if (!tf_broadcaster_) {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(ros_node_);
    }

}

void MyPluginPrivate::OnForceCmd(const geometry_msgs::msg::Wrench::SharedPtr msg){
    lock_guard<mutex> scoped_lock(lock_);
    
    // gazebo::common::Time last_sim_time = world->SimTime();
    // gazebo::common::Time current_sim_time = world->SimTime();
    // double dt = (current_time-last_sim_time).Double();

    fx = msg->force.x;
    fy = msg->force.y;
    fz = msg->force.z;

    // last_sim_time = current_sim_time;

}

void MyPluginPrivate::OnOdomPublish(
    const gazebo::common::Time& current_sim_time,
    const ignition::math::v6::Pose3<double> & pose,
    const ignition::math::v6::Vector3<double> & velocity,
    const ignition::math::v6::Vector3<double> & acceleration
){

    // lock_guard<mutex> scoped_lock(lock_);
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(current_sim_time);
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    
    // position
    odom.pose.pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(pose.Pos());
    odom.pose.pose.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());

    // velocity of the model wrt to local frame(body frame)
    auto linear_global_vel = model->WorldLinearVel();
    auto angular_global_vel = model->WorldAngularVel();
    // getting realtive vels
    auto pose_rot = pose.Rot(); // rot matrix
    auto pose_rot_inv = pose_rot.Inverse();
    auto linear_local_vel = pose_rot_inv.RotateVector(linear_global_vel);
    auto angular_local_vel = pose_rot_inv.RotateVector(angular_global_vel);
    odom.twist.twist.linear = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(linear_local_vel);
    odom.twist.twist.angular = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(angular_local_vel);
    
    cout << odom.pose.pose.position.x << endl;
    cout << odom.pose.pose.position.y << endl;
    cout << odom.pose.pose.position.z << endl;
    cout << pose_rot.Euler() << endl;
    // covariance
    // odom_.pose.covariance[0] = covariance_[0];
    // odom_.pose.covariance[7] = covariance_[1];
    // odom_.pose.covariance[14] = 1000000000000.0;
    // odom_.pose.covariance[21] = 1000000000000.0;
    // odom_.pose.covariance[28] = 1000000000000.0;
    // odom_.pose.covariance[35] = covariance_[2];

    // odom_.twist.covariance[0] = covariance_[0];
    // odom_.twist.covariance[7] = covariance_[1];
    // odom_.twist.covariance[14] = 1000000000000.0;
    // odom_.twist.covariance[21] = 1000000000000.0;
    // odom_.twist.covariance[28] = 1000000000000.0;
    // odom_.twist.covariance[35] = covariance_[2];
    // publishing odom data
    odom_msg_pub_->publish(odom);
    
    // // syncing tf and odom publishing
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = odom.header.stamp;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_footprint";
    tf.transform.translation.x = odom.pose.pose.position.x;
    tf.transform.translation.y = odom.pose.pose.position.y;
    tf.transform.translation.z = odom.pose.pose.position.z;
    tf.transform.rotation = odom.pose.pose.orientation;

    if(tf_broadcaster_){
        tf_broadcaster_->sendTransform(tf);
    }


}

void MyPlugin::Reset(){
  impl_->Reset();
}



GZ_REGISTER_MODEL_PLUGIN(MyPlugin)
}

