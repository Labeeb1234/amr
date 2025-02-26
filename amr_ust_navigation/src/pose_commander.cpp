#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <iostream>
#include <vector>
#include <chrono>

enum Task{
    NONE = 0,
    SPIN = 1,
    NAV_TO_POSE = 2,
    BACKUP = 3,
    COMPUTE_PATH = 4
};



class CommanderNode: public rclcpp::Node{
private:
std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult> future_nav_to_pose_;
std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>::WrappedResult> future_spin_;
std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::BackUp>::WrappedResult> future_backup_;
std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::WrappedResult> future_compute_path_to_pose_;


rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navtp_action_client_;
rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr localization_sub_; // subscription to amcl_pose for setting inital pose of the bot
rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;

rclcpp_action::ResultCode status;
geometry_msgs::msg::PoseStamped initial_pose_;
bool initial_pose_recieved_;
Task current_task;

std::shared_ptr<const void> feedback_ptr;

void on_feedback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
    const std::shared_ptr<nav2_msgs::action::NavigateToPose::Feedback> feedback
){
    RCLCPP_INFO(this->get_logger(), "current bot position: (%f, %f)", feedback->current_pose.pose.position.x, feedback->current_pose.pose.position.y);
}

void on_result(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result){
    switch (result.code){
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Reached Goal!");
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal Plan Aborted");
        break;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Goal Plan Cancelled!");
        break;
    case rclcpp_action::ResultCode::UNKNOWN:
        RCLCPP_ERROR(this->get_logger(), "Unknown error :-|");
        break;
    default:
        RCLCPP_WARN(this->get_logger(), "Unhandled result code: %d", static_cast<int>(result.code));
        break;
    }
}

void wait_for_node_to_activate(const std::string& node_name){
    RCLCPP_INFO(this->get_logger(),"Waiting for %s",node_name.c_str());

    std::string node_service = "/" + node_name + "/get_state";
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client = this->create_client<lifecycle_msgs::srv::GetState>(node_service);

    while (!client->wait_for_service(std::chrono::milliseconds(1000))){
        if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service. Exiting.");
                return;
        }
        RCLCPP_INFO(this->get_logger(),"Waiting for %s to be available", node_name.c_str());
    }

    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    std::string state = "unknown";
    
    while (state != "active" && rclcpp::ok()) {
        auto future = client->async_send_request(request);
        auto status = rclcpp::spin_until_future_complete(
            this->get_node_base_interface(),
            future,
            std::chrono::milliseconds(3000)
        );

        if (status == rclcpp::FutureReturnCode::SUCCESS) {
            state = future.get()->current_state.label;
            RCLCPP_INFO(this->get_logger(), "Node %s state: %s", node_name.c_str(), state.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to get state for %s", node_name.c_str());
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

}

template<typename T> void check_complete(T future){
    if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, std::chrono::milliseconds(100)) == rclcpp::FutureReturnCode::SUCCESS){
        auto result = future.get();
        status = result.code;
    }
}

void set_initial_pose(){
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.pose.pose = initial_pose_.pose;
    msg.header.frame_id = initial_pose_.header.frame_id;
    msg.header.stamp = this->get_clock()->now();
    this->initial_pose_publisher_->publish(msg);
}

void wait_for_initial_pose(){
    RCLCPP_INFO(this->get_logger(),"Waiting for initial pose");
    set_initial_pose();
    while(!initial_pose_recieved_){
        rclcpp::spin_some(this->get_node_base_interface());
    }
}

public:
CommanderNode(): Node("commander_node"){
    initial_pose_.header.frame_id = "map";
    initial_pose_recieved_ = false;
    current_task = NONE;
    status = rclcpp_action::ResultCode::UNKNOWN;
    initial_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);
    localization_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 10, std::bind(&CommanderNode::pose_feedback_callback, this, std::placeholders::_1));

    navtp_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "/navigate_to_pose");


    RCLCPP_INFO(this->get_logger(), "entities created!");
}


void SetInitialPose(const geometry_msgs::msg::Pose::SharedPtr pose){
    RCLCPP_INFO(this->get_logger(), "Setting initial pose...");
    initial_pose_.pose.position = pose->position;
    initial_pose_.pose.orientation = pose->orientation;
    this->initial_pose_recieved_ = false;
    set_initial_pose();
}

void WaitUntilNav2Activated(){
    RCLCPP_INFO(this->get_logger(), "Waiting for NAV2 Stack...");
    wait_for_node_to_activate("amcl");
    wait_for_initial_pose();
    wait_for_node_to_activate("bt_navigator");
    RCLCPP_INFO(this->get_logger(), "NAV2 is active and ready!");
}

bool is_task_completed(){

    if(this->current_task == NONE){
        return true;
    }
    RCLCPP_INFO(get_logger(),"IsTaskComplete waiting for [%d]", current_task);
    
    switch(current_task){
        case SPIN:
            check_complete<std::shared_future<rclcpp_action::Client<nav2_msgs::action::Spin>::WrappedResult>>(this->future_spin_);
            break;
        case NAV_TO_POSE:
            check_complete<std::shared_future<rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::WrappedResult>>(this->future_nav_to_pose_);
            break;
        case BACKUP:
            check_complete<std::shared_future<rclcpp_action::Client<nav2_msgs::action::BackUp>::WrappedResult>>(this->future_backup_);
            break;
        case COMPUTE_PATH:
            check_complete<std::shared_future<rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::WrappedResult>>(this->future_compute_path_to_pose_);
            break;
        default:
            RCLCPP_ERROR(get_logger(),"Undefined task in progress");
    }

    RCLCPP_INFO(this->get_logger(), "Task [%d] completed", current_task);
    if(this->current_task == NONE){
        return true;
    }
    else{
        return false;
    }
}

bool send_target_pose(const geometry_msgs::msg::Pose::SharedPtr pose_msg){

    while (!navtp_action_client_->wait_for_action_server(std::chrono::milliseconds(1000))){
        RCLCPP_ERROR(this->get_logger(), "Waiting for the action server...");
    }

    double roll = 0.0, pitch = 0.0, goal_yaw = 0.0;
    
    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose.pose.position.x = pose_msg->position.x;
    goal_msg.pose.pose.position.y = pose_msg->position.y;
    goal_msg.pose.pose.position.z = 0.0;
    goal_msg.pose.pose.orientation = pose_msg->orientation;
    goal_msg.behavior_tree = "";
    
    tf2::Quaternion q(
        pose_msg->orientation.x,
        pose_msg->orientation.y,
        pose_msg->orientation.z,
        pose_msg->orientation.w
    );
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, goal_yaw);

    RCLCPP_INFO(this->get_logger(), "Setting Goal Pose: [x: %.2f, y: %.2f, yaw: %.2f]", pose_msg->position.x, pose_msg->position.y, goal_yaw);
    RCLCPP_INFO(this->get_logger(), "Sending goal request.....");

    this->current_task = NAV_TO_POSE;
    // goal request options
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&CommanderNode::on_result, this, std::placeholders::_1);


    // setting target pose request
    auto send_goal_future = this->navtp_action_client_->async_send_goal(goal_msg,send_goal_options);
    if(rclcpp::spin_until_future_complete(this->get_node_base_interface(),send_goal_future) != rclcpp::FutureReturnCode::SUCCESS){
        RCLCPP_ERROR(this->get_logger(), "Send goal call failed");
        return false;
    };

    auto goal_handle = send_goal_future.get();
    if(goal_handle != NULL ) {
        if(goal_handle->get_status() != action_msgs::msg::GoalStatus::STATUS_ACCEPTED) {
            RCLCPP_INFO(this->get_logger(),"GoToPose request was rejected");
            this->current_task = NONE;
            return false;
        }
    }
    else {
        RCLCPP_INFO(this->get_logger(),"GoToPose request was rejected");   
        this->current_task = NONE;
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(),"GoToPose request was accepted.");
    future_nav_to_pose_ = navtp_action_client_->async_get_result(goal_handle);
    
    return true;
}

void pose_feedback_callback(const geometry_msgs::msg::PoseWithCovarianceStamped){
    RCLCPP_INFO(this->get_logger(), "Got Pose From AMCL localizer.");
    this->initial_pose_recieved_ = true;
}

~CommanderNode(){}

};


int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto commander_node = std::make_shared<CommanderNode>();

    geometry_msgs::msg::Pose::SharedPtr init_pose = std::make_shared<geometry_msgs::msg::Pose>();
    init_pose->position.x = 0.0;
    init_pose->position.y = 0.0;
    init_pose->orientation.w = 1.0;
    init_pose->orientation.x = 0.0;
    init_pose->orientation.y = 0.0;
    init_pose->orientation.z = 0.0;
    commander_node->SetInitialPose(init_pose);    

    commander_node->WaitUntilNav2Activated();
    
    while(!commander_node->is_task_completed()){}

    geometry_msgs::msg::Pose::SharedPtr goal_pose = std::make_shared<geometry_msgs::msg::Pose>();
    goal_pose->position.x = 0.68;
    goal_pose->position.y = 0.0;
    goal_pose->orientation.w = 1.0;
    goal_pose->orientation.x = 0.0;
    goal_pose->orientation.y = 0.0;
    goal_pose->orientation.z = 0.0;
    commander_node->send_target_pose(goal_pose);

    while(!commander_node->is_task_completed()){}

    rclcpp::spin(commander_node);
    rclcpp::shutdown();

    return 0;
}