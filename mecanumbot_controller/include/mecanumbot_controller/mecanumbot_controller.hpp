// this controller is just uses basic open loop feedfwd inverse kinematics to control a 4 wheel mecanum drive robot
// the controller is build based on the lifecycle node concepts used in the ros2 control documentation 
// The code is just a basic templete codes for odom frame addition need to be done (reminder for me )

// note the visibility.h library is optional you can add it if you want it to make the code function better although I really didn't observe much difference 
// need to modify accordingly to be put in IssacSim (will do)
#ifndef MECANUM_BOT_CONTROLLER__MECANUM_BOT_CONTROLLER_HPP_
#define MECANUM_BOT_CONTROLLER__MECANUM_BOT_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


namespace mecanumbot_controller
{
// inheritance 
class MecanumBotController: public controller_interface::ControllerInterface
{
    // using unstamped twist command since by default the ros2 humble teleop packages only sent upstamped msgs to the robot
    // need to look or make teleop for sending stamped msgs
    // using Twist variable for unstamped twist msgs
    using Twist = geometry_msgs::msg::Twist;
        
    // public attributes
public:
    // constructor
    MecanumBotController();
           
    // virtual fuctions to be overridden
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
            
    // the update state is where our main code/ kinematics will be looped or run by the code 
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    // optional we can use the init() instead of on_init() function which is of the return_type() refer controller_interface library of ros2_control
    controller_interface::CallbackReturn on_init() override;
    // states will be shown when we launch the code via the launch file which launchs the gazebo along with the robot using spawn node
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

    // Desctructor
    ~MecanumBotController();

protected: // can be accesed by derived classes

    // need to create wheel/Rim handles stuct through which the commands are send to the robot wheels and states of teh robot are recorded
    // since we are using wheeled mobile robots the command_interface will be velocity and state interface will be postion and velocity
    struct RimHandle
    {   
        // use referecnce wrappers to store the state_interfaces and command_interfaces data incase theres are mutilple wheels this is better
        std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state;
        std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_state;
        // for mobile robots use velocity_command interface 
        std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_command;
    };

    // add protected functions and variables you need to be used in the main controller cpp code 
    // use vectors to store the rim names and rim joint names (joint names != rim names not necessarily check urdf)
    std::vector<std::string> rim_joint_names_;
    std::vector<RimHandle> registered_rim_handles_; // state and command interfaces
    // attributes to initialize the subscriber in the main cpp code 
    
    bool subscriber_is_active_ = false;
    // unstamped velocity_command_subcriber (shared pointer variable)
    rclcpp::Subscription<Twist>::SharedPtr velocity_command_subscriber_;
    realtime_tools::RealtimeBuffer<std::shared_ptr<Twist>> velocity_command_ptr_; // need to check more on realtimetools pkg of ros2_control


    // physical attributes of the robot that can be generalized
    double rim_radius_;
    double rim_separation_length_;
    double rim_separation_width_;

    // storing the timestamp of previous published msg
    rclcpp::Time previous_updated_timestamp_{0};
    rclcpp::Time previous_time_stamp_{0};
    

    // previous updated time for pid implementation;
    // rclcpp::Time previousPid_timestampe_{0};

    double publish_rate_ = 50.0;
    rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);
    rclcpp::Time previous_publish_timestamp_{0, 0, RCL_CLOCK_UNINITIALIZED};

    //bool use_stamped_vel;
    //bool reset();

};
}

#endif