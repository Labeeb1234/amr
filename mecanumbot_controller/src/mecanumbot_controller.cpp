#include <memory> 
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "mecanumbot_controller/mecanumbot_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "tf2/LinearMath/Quaternion.h"


// some variables can be initialized here which can be changed here instead of going into the code 
namespace
{
    constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel_unstamped";
    constexpr auto DEFAULT_ODOM_TOPIC = "~/odom";
    //constexpr auto DEFAULT_TRANSFOR_TOPIC = "~/tf2"; //(optional)
    constexpr int MAX_WHEEL_NUMBER = 4;
}

namespace mecanumbot_controller
{
    //using namespace std::chrono_literals;
    using controller_interface::interface_configuration_type;
    using controller_interface::InterfaceConfiguration;
    using hardware_interface::HW_IF_POSITION;
    using hardware_interface::HW_IF_VELOCITY;
    using lifecycle_msgs::msg::State;

    MecanumBotController::MecanumBotController() 
        : controller_interface::ControllerInterface()
        , velocity_command_subscriber_(nullptr)
        , velocity_command_ptr_(nullptr){}

    // intialization phase of the controller
    controller_interface::CallbackReturn MecanumBotController::on_init()
    {
        RCLCPP_INFO(get_node()->get_logger(), "Entered the initialization phase\n");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    // command_interface for each rim_joints (hence interface_type will be individual)
    InterfaceConfiguration MecanumBotController::command_interface_configuration() const
    {
        RCLCPP_INFO(get_node()->get_logger(), "Specifying command interface for each joint\n");
        std::vector<std::string> conf_names;
        for(auto &joint_name: rim_joint_names_)
        {
            conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
        }
        return {interface_configuration_type::INDIVIDUAL, conf_names};
    }

    // just like the above for each rim_joint specify the state inteface type
    InterfaceConfiguration MecanumBotController::state_interface_configuration() const
    {
        RCLCPP_INFO(get_node()->get_logger(), "Specifying state interface for each joint\n");
        std::vector<std::string> conf_names;
        for(auto &joint_name: rim_joint_names_)
        {
            conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
            conf_names.push_back(joint_name + "/" + HW_IF_POSITION);
        }
        return {interface_configuration_type::INDIVIDUAL, conf_names};
    }

    // configuration state of the controller
    controller_interface::CallbackReturn MecanumBotController::on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Configuring controller and getting parameters\n");
        // parameter for robot active joints
        rim_joint_names_ = get_node()->get_parameter("rim_joint_names").as_string_array();

        if(rim_joint_names_.size() > MAX_WHEEL_NUMBER)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "The number of wheels on the robot is more the allowed number!\n");
            return controller_interface::CallbackReturn::ERROR;
        }
        if(rim_joint_names_.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "No rim joint names provided\n");
            return controller_interface::CallbackReturn::ERROR;
        }
        // paramter for rim/wheel radius
        rim_radius_ = get_node()->get_parameter("rim_radius").as_double();
        if(rim_radius_ < 0)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "The provided radius is negative\n");
            return controller_interface::CallbackReturn::ERROR;
        }
        // parameter for rim/wheel separation 
        rim_separation_length_ = get_node()->get_parameter("rim_separation_length").as_double();
        if(rim_separation_length_ < 0)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "The provided wheel separation length is negative\n");
            return controller_interface::CallbackReturn::ERROR;
        }
        // parameter for rim/wheel separation 
        rim_separation_width_ = get_node()->get_parameter("rim_separation_width").as_double();
        if(rim_separation_width_ < 0)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "The provided wheel separation width is negative\n");
            return controller_interface::CallbackReturn::ERROR;
        }

        // odom_params_.odom_frame_id = get_node()->get_parameter("odom_frame_id").as_string();
        // odom_params_.base_frame_id = get_node()->get_parameter("base_frame_id").as_string();
        // odom_params_.open_loop = get_node()->get_parameter("open_loop").as_bool();
        // odom_params_.enable_odom_tf = get_node()->get_parameter("enable_odom_tf").as_bool();

        // initializing odom publisher
        // odom_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(DEFAULT_ODOM_TOPIC, rclcpp::SystemDefaultsQoS());

        // publish rate paramter
        publish_rate_ = get_node()->get_parameter("publish_rate").as_double();
        publish_period_ = rclcpp::Duration::from_seconds(1.0/publish_rate_);
        previous_publish_timestamp_ = get_node()->get_clock()->now();

        // creating a subcriber to subcriber to Twist(unstamped) topic
        velocity_command_subscriber_ = get_node()->create_subscription<Twist>("amr_ust/cmd_vel", rclcpp::SystemDefaultsQoS(),[this](Twist::SharedPtr msg)
            {
                if(!subscriber_is_active_)
                {
                    RCLCPP_WARN(get_node()->get_logger(),"Can't accept new msgs subscriber is not active\n");
                    return;
                }
                
                velocity_command_ptr_.writeFromNonRT(msg);
                //velocity_command_ptr_.get(msg);
                // fake headers
                //std::shared_ptr<Twist> twist_stamped;
                //velocity_command_ptr_.get(twist_stamped);
                //twist_stamped->twist = *msg;
                //twist_stamped->header.stamp = get_node()->get_clock()->now();
                
            });

        return controller_interface::CallbackReturn::SUCCESS;
    }

    // Activation phase of the controller
    // here each of the wheel handle need to get its interface specified quite important
    // CAUTION make sure the the wheel handle structure order is maintained and each individual joint gets its command and state interfaces
    controller_interface::CallbackReturn MecanumBotController::on_activate(const rclcpp_lifecycle::State &)
    {
        if(rim_joint_names_.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "rim joint name are empty\n");
            return controller_interface::CallbackReturn::ERROR;
        }

        registered_rim_handles_.reserve(rim_joint_names_.size());
        {
            for(const auto &rim_joint_name: rim_joint_names_)
            {
                const auto interface_name = HW_IF_VELOCITY;
                const auto state_handle = std::find_if(
                    state_interfaces_.cbegin(), state_interfaces_.cend(), [&rim_joint_name, &interface_name](const auto &interface)
                    {
                        return interface.get_prefix_name() == rim_joint_name && interface.get_interface_name() == interface_name;

                    });

                if(state_handle == state_interfaces_.cend())
                {
                    RCLCPP_ERROR(get_node()->get_logger(), "Unable to get joint state handle for %s", rim_joint_name.c_str());
                    return controller_interface::CallbackReturn::ERROR;
                }

                const auto pos_interface_name = HW_IF_POSITION;
                const auto pos_handle = std::find_if(
                    state_interfaces_.cbegin(), state_interfaces_.cend(), [&rim_joint_name, &pos_interface_name](const auto &pos_interface)
                    {
                        return pos_interface.get_prefix_name() == rim_joint_name && pos_interface.get_interface_name() == pos_interface_name;

                    });
                if(pos_handle == state_interfaces_.cend())
                {
                    RCLCPP_ERROR(get_node()->get_logger(), "Unable to get joint position handle for %s", rim_joint_name.c_str());
                    return controller_interface::CallbackReturn::ERROR;
                }

                const auto command_handle = std::find_if(
                    command_interfaces_.begin(), command_interfaces_.end(), [&rim_joint_name](const auto &interface)
                    {
                        return interface.get_prefix_name() == rim_joint_name && interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
                    });
                
                if(command_handle == command_interfaces_.end())
                {
                    RCLCPP_ERROR(get_node()->get_logger(), "Unable to get joint command handle for %s", rim_joint_name.c_str());
                    return controller_interface::CallbackReturn::ERROR;
                }

                registered_rim_handles_.emplace_back(RimHandle{std::ref(*state_handle), std::ref(*pos_handle), std::ref(*command_handle)});

                RCLCPP_INFO(get_node()->get_logger(),"Got command interface: %s", command_handle->get_name().c_str());
                RCLCPP_INFO(get_node()->get_logger(),"Got state interface: %s", state_handle->get_name().c_str());
                RCLCPP_INFO(get_node()->get_logger(),"Got position interface: %s", pos_handle->get_name().c_str());
       
            }
            // activate the subscriber now 
            subscriber_is_active_ = true;
            RCLCPP_INFO(get_node()->get_logger(),"Subscriber and publisher are active now\n");
            previous_updated_timestamp_ = get_node()->get_clock()->now();
            return controller_interface::CallbackReturn::SUCCESS;  
        }      
    }

    // Deactivate phase
    controller_interface::CallbackReturn MecanumBotController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_DEBUG(get_node()->get_logger(), "Called on_deactivate\n");
        // subscriber inactive
        subscriber_is_active_ = false;
        // clear out velocity command data from rim handles
        registered_rim_handles_.clear();
        return controller_interface::CallbackReturn::SUCCESS;

    }
    // cleanup phase (add required actions here)
    controller_interface::CallbackReturn  MecanumBotController::on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_DEBUG(get_node()->get_logger(),"Called on_cleanup\n");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    // if errors occur what to do ?
    controller_interface::CallbackReturn  MecanumBotController::on_error(const rclcpp_lifecycle::State &)
    {
        RCLCPP_DEBUG(get_node()->get_logger(),"Called on_error\n");
        return controller_interface::CallbackReturn::SUCCESS; 
    }

    // shutdown phase of the controllers (reseting data can be done here)
    controller_interface::CallbackReturn  MecanumBotController::on_shutdown(const rclcpp_lifecycle::State &)
    {
        RCLCPP_DEBUG(get_node()->get_logger(),"Called on_shutdown\n");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    // update phase where our main action is run here and the kinematics and all are added here
    controller_interface::return_type MecanumBotController::update(const rclcpp::Time &, const rclcpp::Duration &)
    {

        // to get previous velocity command 
        auto velocity_command = velocity_command_ptr_.readFromRT();
        if (!velocity_command || !(*velocity_command)) 
        {
            return controller_interface::return_type::OK;
        }

        // accessing data from unstamped twist msg 
        const auto msg_linear = (*velocity_command)->linear;
        const auto msg_angular = (*velocity_command)->angular;

        // desired ROBOT VELOCITY msg
        double u = msg_linear.x;
        double v = msg_linear.y;
        double r = msg_angular.z;

        // wheel angular velocity calculated from desired robot velocity
        std::vector<double> wheel_velocity(4);
        wheel_velocity[0] = (1/rim_radius_)*(u + v + r*((rim_separation_length_- rim_separation_width_)/2)); // revolute-1
        wheel_velocity[1] = (1/rim_radius_)*(u - v - r*((rim_separation_length_- rim_separation_width_)/2)); // revolute-2
        wheel_velocity[2] = (1/rim_radius_)*(u - v + r*((rim_separation_length_- rim_separation_width_)/2)); // revolute-3
        wheel_velocity[3] = (1/rim_radius_)*(u + v - r*((rim_separation_length_- rim_separation_width_)/2)); // revolute-4
        
        registered_rim_handles_[0].velocity_command.get().set_value(wheel_velocity[0]);
        registered_rim_handles_[1].velocity_command.get().set_value(wheel_velocity[1]);
        registered_rim_handles_[2].velocity_command.get().set_value(wheel_velocity[2]);
        registered_rim_handles_[3].velocity_command.get().set_value(wheel_velocity[3]);

        return controller_interface::return_type::OK;
    }


    MecanumBotController::~MecanumBotController() {}

}

// very important
#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  mecanumbot_controller::MecanumBotController, controller_interface::ControllerInterface)
