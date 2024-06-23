#include "gazebo/common/Time.hh"
#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"
#include "gazebo_ros/conversions/builtin_interfaces.hpp"
#include "gazebo_ros/conversions/geometry_msgs.hpp"
#include "gazebo_ros/node.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sdf/sdf.hh"
#include "rclcpp/rclcpp.hpp"


#include "mecanum_drive_controller/mecanum_drive_controller.hpp"


#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <thread>
#include <stdio.h>


namespace gz_plugins
{

class MecanumDriveControllerPrivate{
  public:
  /// Indicates where the odometry info is coming from
  enum OdomSource
  {
    /// Use an ancoder
    ENCODER = 0,

    /// Use ground truth from simulation world
    WORLD = 1,
  };


  /// Callback to be called at every simulation iteration.
  /// \param[in] _info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo & _info);

  /// Callback when a velocity command is received.
  /// \param[in] _msg Twist command message.
  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg);

  /// Update wheel velocities according to latest target velocities.
  void UpdateWheelVelocities();

  /// Update odometry according to encoder.
  /// \param[in] _current_time Current simulation time
  void UpdateOdometryEncoder(const gazebo::common::Time & _current_time);

  /// Update odometry according to world
  void UpdateOdometryWorld();
  
  /// Publish odometry messages
  /// \param[in] _current_time Current simulation time
  void PublishOdometryMsg(const gazebo::common::Time & _current_time);

  /// Publish odometry transforms
  /// \param[in] _current_time Current simulation time
  void PublishOdometryTf(const gazebo::common::Time & _current_time);

  /// Publish trasforms for the wheels
  /// \param[in] _current_time Current simulation time
  void PublishWheelsTf(const gazebo::common::Time & _current_time);

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Subscriber to command velocities
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  /// Odometry publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

  /// Connection to event called at every world iteration.
  gazebo::event::ConnectionPtr update_connection_;

  /// Distance between the wheels in width, in meters.
  std::vector<double> wheel_separation_width_;

  // Distance between the wheels in length in meters.
  std::vector<double> wheel_separation_length_;

  /// Diameter of wheels, in meters.
  std::vector<double> wheel_diameter_;

  /// Maximum wheel torque, in Nm.
  double max_wheel_torque_;

  /// Maximum wheel acceleration
  double max_wheel_accel_;

  /// Desired wheel speed.
  std::vector<double> desired_wheel_speed_;

  /// Speed sent to wheel.
  std::vector<double> wheel_speed_instr_;

  /// Pointers to wheel joints.
  std::vector<gazebo::physics::JointPtr> joints_;

  /// Pointer to model.
  gazebo::physics::ModelPtr model_;

  /// To broadcast TFs
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  /// Protect variables accessed on callbacks.
  std::mutex lock_;

  /// Linear velocity in X received on command (m/s).
  double target_x_{0.0};

  /// longitudnal velocity in Y received on command (m/s).
  double target_y_{0.0};

  /// Angular velocity in Z received on command (rad/s).
  double target_rot_{0.0};

  /// Update period in seconds.
  double update_period_;

  /// Last update time.
  gazebo::common::Time last_update_time_;

  /// Keep encoder data.
  geometry_msgs::msg::Pose2D pose_encoder_;

  /// Odometry frame ID
  std::string odometry_frame_;

  /// Last time the encoder was updated
  gazebo::common::Time last_encoder_update_;

  /// Either ENCODER or WORLD
  OdomSource odom_source_;

  /// Keep latest odometry message
  nav_msgs::msg::Odometry odom_;

  /// Robot base frame ID
  std::string robot_base_frame_;

  /// True to publish odometry messages.
  bool publish_odom_;

  /// True to publish wheel-to-base transforms.
  bool publish_wheel_tf_;

  /// True to publish odom-to-world transforms.
  bool publish_odom_tf_;

  /// Store number of wheels
  unsigned int num_wheels_;

  /// Covariance in odometry
  double covariance_[3];

  /// PID parameters
  double kp;


};

MecanumDriveController::MecanumDriveController(): impl_(std::make_unique<MecanumDriveControllerPrivate>()){}
MecanumDriveController::~MecanumDriveController(){}

void MecanumDriveController::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf){
  // getting model
  impl_->model_ = _model;
  
  // initializing ros(2)_node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  // parameter to define the numeber of wheels on the bot
  impl_->num_wheels_ = static_cast<unsigned int>(_sdf->Get<int>("number_of_wheels", 1).first);

  // error debugginh in case of invalid num_wheel param specified in URDF
  if(impl_->num_wheels_ < 1){
    impl_->num_wheels_ = 1;
    RCLCPP_WARN(impl_->ros_node_->get_logger(), "Invalid number of wheels specified");
    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Setting wheel number to [%d]", impl_->num_wheels_);
  }

  // setting the dynamic properties of the model(acceleration and torque to the wheels/wheel joints)
  impl_->max_wheel_torque_ = _sdf->Get<double>("max_torque", 100.0).first; // in[Nm]
  impl_->max_wheel_accel_= _sdf->Get<double>("max_acceleration", 0.0).first; // in [rad/s^2]

  // getting the joint/kinematic parameters
  std::vector<gazebo::physics::JointPtr> joints;
  for(auto joint_elem = _sdf->GetElement("joint"); joint_elem != nullptr; joint_elem = joint_elem->GetNextElement("joint")){
    auto joint_name = joint_elem->Get<std::string>();

    auto joint = _model->GetJoint(joint_name);

    if(!joint){
      RCLCPP_ERROR(impl_->ros_node_->get_logger(),"Joint [%s] not found, plugin will not work.", joint_name.c_str());
      impl_->ros_node_.reset();
      return;
    }
    joint->SetParam("fmax", 0, impl_->max_wheel_torque_);
    RCLCPP_INFO(impl_->ros_node_->get_logger(), "joint_name: %s", joint_name.c_str());
    joint->SetParam("fmax", 0, impl_->max_wheel_torque_); // setting required motor torque
    RCLCPP_INFO(impl_->ros_node_->get_logger(), "setting joint [%s] torque as %f [Nm]", joint_name.c_str(), impl_->max_wheel_torque_); 
    joints.push_back(joint);

  }

  // RCLCPP_INFO(impl_->ros_node_->get_logger(), "works till here....!");

  unsigned int index = 0;
  // populating joint pointer informations
  for(index = 0; index < impl_->num_wheels_; index++){
    impl_->joints_.push_back(joints[index]);
  }

  index = 0;
  // setting the wheel separaion width (2*w) in metres[m]
  unsigned int wheel_pairs = (impl_->num_wheels_)/2;
  impl_->wheel_separation_width_.assign(wheel_pairs, 0.34);
  for(auto wheel_width = _sdf->GetElement("wheel_separation_width"); wheel_width != nullptr; wheel_width = wheel_width->GetNextElement("wheel_separation_width")){
    if (index >= wheel_pairs) {
      RCLCPP_WARN(impl_->ros_node_->get_logger(), "Ignoring rest of specified <wheel_separation_width>");
      break;
    }
    impl_->wheel_separation_width_[index] = wheel_width->Get<double>();
    RCLCPP_INFO(impl_->ros_node_->get_logger(),"Wheel pair %i separation set to [%fm]", index + 1, impl_->wheel_separation_width_[index]);
    index++;

  }
  // setting wheel separation length (2*l) in metres[m]
  index = 0;
  impl_->wheel_separation_length_.assign(wheel_pairs, 0.34);
  for(auto wheel_width = _sdf->GetElement("wheel_separation_length"); wheel_width!=nullptr; wheel_width= wheel_width->GetNextElement("wheel_separation_length")){
    if (index >= wheel_pairs) {
      RCLCPP_WARN(impl_->ros_node_->get_logger(), "Ignoring rest of specified <wheel_separation_length>");
      break;
    }
    impl_->wheel_separation_length_[index] = wheel_width->Get<double>();
    RCLCPP_INFO(impl_->ros_node_->get_logger(),"Wheel pair %i separation set to [%fm]", index + 1, impl_->wheel_separation_length_[index]);
    index++;
  }
  // setting wheel diameter (d) in metres[m]
  index = 0;
  impl_->wheel_diameter_.assign(impl_->num_wheels_, 0.6);
  for(auto diameter = _sdf->GetElement("wheel_diameter"); diameter!=nullptr; diameter = diameter->GetNextElement("wheel_diameter")){
    
    if (index >= impl_->num_wheels_) {
      RCLCPP_WARN(impl_->ros_node_->get_logger(), "Ignoring rest of specified <wheel_diameter>");
      break;
    }
    
    impl_->wheel_diameter_[index] = diameter->Get<double>();
    RCLCPP_INFO(impl_->ros_node_->get_logger(),"Wheel %i diameter set to [%fm]", index + 1, impl_->wheel_diameter_[index]); 
    index++;
  }

  // setting wheel speed parameters
  impl_->wheel_speed_instr_.assign(impl_->num_wheels_, 0); // not used for now
  impl_->desired_wheel_speed_.assign(impl_->num_wheels_, 0);

  // setting Update rate
  auto update_rate = _sdf->Get<double>("update_rate", 100.0).first;
  if (update_rate > 0.0) {
    impl_->update_period_ = 1.0 / update_rate;
  } else {
    impl_->update_period_ = 0.0;
  }
  impl_->last_update_time_ = _model->GetWorld()->SimTime(); // recording last simulation time 

  // twist msgs subscriptions
  impl_->cmd_vel_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", qos.get_subscription_qos("cmd_vel", rclcpp::QoS(1)),
    std::bind(&MecanumDriveControllerPrivate::OnCmdVel, impl_.get(), std::placeholders::_1));
  
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Subscribed to [%s]",impl_->cmd_vel_sub_->get_topic_name());

  // definign odometry parameters
  // Odometry
  impl_->odometry_frame_ = _sdf->Get<std::string>("odometry_frame", "odom").first;
  impl_->robot_base_frame_ = _sdf->Get<std::string>("robot_base_frame", "base_footprint").first;
  impl_->odom_source_ = static_cast<MecanumDriveControllerPrivate::OdomSource>(_sdf->Get<int>("odometry_source", 1).first);

  // Advertise odometry topic
  impl_->publish_odom_ = _sdf->Get<bool>("publish_odom", false).first;
  if (impl_->publish_odom_) {
    impl_->odometry_pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>("odom", qos.get_publisher_qos("odom", rclcpp::QoS(1)));
    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Advertise odometry on [%s]",impl_->odometry_pub_->get_topic_name());
  }

  // Create TF broadcaster if needed
  index = 0;
  impl_->publish_wheel_tf_ = _sdf->Get<bool>("publish_wheel_tf", false).first;
  impl_->publish_odom_tf_ = _sdf->Get<bool>("publish_odom_tf", false).first;
  if (impl_->publish_wheel_tf_ || impl_->publish_odom_tf_) {
    impl_->transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(impl_->ros_node_);

    if (impl_->publish_odom_tf_) {
      RCLCPP_INFO(impl_->ros_node_->get_logger(),"Publishing odom transforms between [%s] and [%s]", impl_->odometry_frame_.c_str(),impl_->robot_base_frame_.c_str());
    }

    for (index = 0; index < impl_->num_wheels_; index++) {
      if (impl_->publish_wheel_tf_) {
        RCLCPP_INFO(impl_->ros_node_->get_logger(),"Publishing wheel transforms between [%s], [%s]",impl_->robot_base_frame_.c_str(),impl_->joints_[index]->GetName().c_str());
      }
    }
  }

  // setting covariance parameters for bot position
  impl_->covariance_[0] = _sdf->Get<double>("covariance_x", 0.00001).first;
  impl_->covariance_[1] = _sdf->Get<double>("covariance_y", 0.00001).first;
  impl_->covariance_[2] = _sdf->Get<double>("covariance_yaw", 0.00001).first;

  impl_->kp = _sdf->Get<double>("kp", 10.0).first;
  
  if(!_sdf->HasElement("kp")){
    impl_->kp=1.0;
    RCLCPP_INFO(impl_->ros_node_->get_logger(),"proportinal controller value not specified in xml file setting kp to [%f]", impl_->kp);
  }
  else{
    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Setting proportinal controller value as: [%f]", impl_->kp);
  }


  // Listen to the update event (broadcast every simulation iteration) --> very important part
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&MecanumDriveControllerPrivate::OnUpdate, impl_.get(), std::placeholders::_1));  

}

// populating reset function under the MecanumDriveController class
void MecanumDriveController::Reset(){

  impl_->last_update_time_ = impl_->joints_[0]->GetWorld()->SimTime();
  for (unsigned int i = 0; i < impl_->num_wheels_; i++) {
    if (impl_->joints_[i]){
      impl_->joints_[i]->SetParam("fmax", 0, impl_->max_wheel_torque_);
    }
  }
  impl_->pose_encoder_.x = 0;
  impl_->pose_encoder_.y = 0;
  impl_->pose_encoder_.theta = 0;
  impl_->target_x_ = 0;
  impl_->target_rot_ = 0;

}


// main event update callback function
void MecanumDriveControllerPrivate::OnUpdate(const gazebo::common::UpdateInfo & _info){
  #ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE("MecanumDriveControllerPrivate::OnUpdate");
  #endif

  // update encoder reading 
  if(odom_source_ == ENCODER){
    UpdateOdometryEncoder(_info.simTime);
  }

  double dt  = (_info.simTime - last_update_time_).Double();
  if(dt < update_period_){
    return;
  }

  // update gazebo pose onto odom msgs
  #ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_BEGIN("UpdateOdometryWorld");
  #endif
  if(odom_source_ == WORLD){
    UpdateOdometryWorld();
  }

  #ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
    IGN_PROFILE_BEGIN("PublishOdometryMsg");
  #endif
  if(publish_odom_){
    PublishOdometryMsg(_info.simTime);
  }

  // update and publish odom->robot_base_frame tf
  #ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
    IGN_PROFILE_BEGIN("PublishOdometryTf");
  #endif
  if(publish_odom_tf_){
    PublishOdometryTf(_info.simTime);
  }

  // update and publish wheel tf
  #ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
    IGN_PROFILE_BEGIN("PublishWheelsTf");
  #endif
  if(publish_wheel_tf_){
    PublishWheelsTf(_info.simTime);
  }

  #ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
    IGN_PROFILE_BEGIN("UpdateWheelVelocites");
  #endif
  UpdateWheelVelocities();

  #ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
  #endif

  // setting current wheel speeds
  std::vector<double> current_wheel_speed(num_wheels_);
  for(unsigned int i = 0; i < num_wheels_; i++){
    current_wheel_speed[i] = joints_[i]->GetVelocity(0); // current angular wheel velocites in [rad/s]
    // std::cout << "wheel speed of joint ["<< joints_[i]->GetName() << "]: " << current_wheel_speed[i] << std::endl;
    // std::cout << "desired wheel speed of joint ["<< i+1 << "]: " << desired_wheel_speed_[i] << std::endl; 
  }

  // once the target speed is reached for a wheel
  for(unsigned int i = 0; i < num_wheels_; i++){
    // inertia based brake application
    if(desired_wheel_speed_[i] == 0.0){
      model_->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
      model_->SetAngularVel(ignition::math::Vector3d(0, 0, 0));
      joints_[i]->SetParam("vel", 0, 0.0);
      current_wheel_speed[i]=0.0;
      wheel_speed_instr_[i]=0.0;
    }

    // both desired_wheel_speeds and current_wheel_speeds are in rad/s
    double speed_diff = fabs(desired_wheel_speed_[i] - current_wheel_speed[i]);

    // Set the velocity parameter of the joint
    std::cout << "max_wheel_acceleration: " << max_wheel_accel_ << "rad/ss" << std::endl;
    if(max_wheel_accel_ == 0 || (speed_diff)<0.01){
      joints_[i]->SetParam("vel", 0, desired_wheel_speed_[i]);
    }
    else{
      if(desired_wheel_speed_[i]-current_wheel_speed[i]>=0){
        wheel_speed_instr_[i]+=fmin(desired_wheel_speed_[i]-current_wheel_speed[i], max_wheel_accel_*dt);
      }
      else{
        wheel_speed_instr_[i]+=fmax(desired_wheel_speed_[i]-current_wheel_speed[i], -max_wheel_accel_*dt);
      }
      joints_[i]->SetParam("vel", 0, wheel_speed_instr_[i]);
    }

    std::cout << "current wheel speed of joint [" << joints_[i]->GetName() << "]: " << current_wheel_speed[i] << std::endl;
    std::cout << "desired wheel speed of joint ["<< joints_[i]->GetName() << "]: " << desired_wheel_speed_[i] << std::endl; 
    std::cout << "instructed wheel speed of joint ["<< joints_[i]->GetName() << "]: " << wheel_speed_instr_[i] << std::endl; 

  }

  last_update_time_ = _info.simTime;

}


void MecanumDriveControllerPrivate::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg){
  std::lock_guard<std::mutex> scoped_lock(lock_);

  target_x_ = _msg->linear.x;
  target_y_ = _msg->linear.y;
  target_rot_ = _msg->angular.z;

}

void MecanumDriveControllerPrivate::UpdateWheelVelocities(){
  std::lock_guard<std::mutex> scoped_lock(lock_);

  // inverse kinematics
  double u = target_x_;
  double v = target_y_;
  double r = target_rot_;
  
  desired_wheel_speed_[0] = (2/wheel_diameter_[0])*(u + v + r*((wheel_separation_length_[0]- wheel_separation_width_[0])/2)); // revolute-1
  desired_wheel_speed_[1] = (2/wheel_diameter_[1])*(u - v - r*((wheel_separation_length_[1]- wheel_separation_width_[1])/2)); // revolute-2
  desired_wheel_speed_[2] = (2/wheel_diameter_[2])*(u - v + r*((wheel_separation_length_[0]- wheel_separation_width_[0])/2)); // revolute-3
  desired_wheel_speed_[3] = (2/wheel_diameter_[3])*(u + v - r*((wheel_separation_length_[1]- wheel_separation_width_[1])/2)); // revolute-4

}

// populating odom msgs on odom topic from wheel encoder readings
void MecanumDriveControllerPrivate::UpdateOdometryEncoder(const gazebo::common::Time & _current_time){
  // can be optimized using 
  double v_fr = (wheel_diameter_[0]/2)*(joints_[0]->GetVelocity(0));
  double v_fl = (wheel_diameter_[1]/2)*(joints_[1]->GetVelocity(0));
  double v_rr = (wheel_diameter_[2]/2)*(joints_[2]->GetVelocity(0));
  double v_rl = (wheel_diameter_[3]/2)*(joints_[3]->GetVelocity(0));

  double dt = (_current_time-last_encoder_update_).Double();
  last_encoder_update_ = _current_time;

  double l = wheel_separation_length_[0]/2;
  double w = wheel_separation_width_[0]/2;
  
  double delta_u = ((v_fr + v_fl + v_rr + v_rl)/4)*dt;
  double delta_v = ((v_fr - v_fl - v_rr + v_rl)/4)*dt;
  double dtheta = ((v_fr - v_fl + v_rr - v_rl)/4)*(1/(l-w))*dt;

  double dx = cos(pose_encoder_.theta + dtheta)*delta_u - sin(pose_encoder_.theta + dtheta)*delta_v;
  double dy = cos(pose_encoder_.theta +dtheta)*delta_v - sin(pose_encoder_.theta + dtheta)*delta_u;

  pose_encoder_.x += dx;
  pose_encoder_.y += dy;
  pose_encoder_.theta += dtheta;

  tf2::Quaternion qt;
  tf2::Vector3 vt;
  qt.setRPY(0.0, 0.0, pose_encoder_.theta);
  vt = tf2::Vector3(pose_encoder_.x, pose_encoder_.y, 0.0);

  odom_.pose.pose.position.x = vt.x();
  odom_.pose.pose.position.y = vt.y();
  odom_.pose.pose.position.z = vt.z(); // optional
  odom_.pose.pose.orientation.x = qt.x();
  odom_.pose.pose.orientation.y = qt.y();
  odom_.pose.pose.orientation.z = qt.z();
  odom_.pose.pose.orientation.w = qt.w();
  odom_.twist.twist.linear.x = delta_u/dt;
  odom_.twist.twist.linear.y = delta_v/dt;
  odom_.twist.twist.angular.z = dtheta/dt;


}

// populating odom msgs on odom topic ,Odometry pose from Gazebo Positioning System (model pose)
void MecanumDriveControllerPrivate::UpdateOdometryWorld(){
  auto pose = model_->WorldPose();
  odom_.pose.pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(pose.Pos());
  odom_.pose.pose.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());

  // getting velocity in odom frame (global velocity variables)
  auto linear = model_->WorldLinearVel();
  odom_.twist.twist.angular.z = model_->WorldAngularVel().Z();

  // robot frame velocities
  double yaw_angle = pose.Rot().Yaw();
  odom_.twist.twist.linear.x = cosf(yaw_angle)*linear.X() + sinf(yaw_angle)*linear.Y();
  odom_.twist.twist.linear.y = cosf(yaw_angle)*linear.Y() - sinf(yaw_angle)*linear.X();

}

// odometry msg publisher
void MecanumDriveControllerPrivate::PublishOdometryMsg(const gazebo::common::Time& _current_time){
  // Set covariance
  odom_.pose.covariance[0] = covariance_[0];
  odom_.pose.covariance[7] = covariance_[1];
  odom_.pose.covariance[14] = 1000000000000.0;
  odom_.pose.covariance[21] = 1000000000000.0;
  odom_.pose.covariance[28] = 1000000000000.0;
  odom_.pose.covariance[35] = covariance_[2];

  odom_.twist.covariance[0] = covariance_[0];
  odom_.twist.covariance[7] = covariance_[1];
  odom_.twist.covariance[14] = 1000000000000.0;
  odom_.twist.covariance[21] = 1000000000000.0;
  odom_.twist.covariance[28] = 1000000000000.0;
  odom_.twist.covariance[35] = covariance_[2];

  // setting header
  odom_.header.frame_id = odometry_frame_;
  odom_.child_frame_id = robot_base_frame_;
  odom_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);

  // publishing odom msg to the odomtopic
  odometry_pub_->publish(odom_);

}

// odom --> base_footprint tf broadcaster
void MecanumDriveControllerPrivate::PublishOdometryTf(const gazebo::common::Time& _current_time){
  
  geometry_msgs::msg::TransformStamped msg;

  msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
  msg.header.frame_id = odometry_frame_;
  msg.child_frame_id = robot_base_frame_;
  msg.transform.translation = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(odom_.pose.pose.position);
  msg.transform.rotation = odom_.pose.pose.orientation;
  transform_broadcaster_->sendTransform(msg);

}

// relative wheel transforms
void MecanumDriveControllerPrivate::PublishWheelsTf(const gazebo::common::Time& _current_time){
  for(unsigned int i=0; i < num_wheels_; i++){
    auto rim_pose = joints_[i]->GetChild()->RelativePose();
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
    msg.header.frame_id = joints_[i]->GetParent()->GetName();
    msg.child_frame_id = joints_[i]->GetChild()->GetName();
    msg.transform.translation = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(rim_pose.Pos());
    msg.transform.rotation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(rim_pose.Rot());

    transform_broadcaster_->sendTransform(msg);
  }

}

GZ_REGISTER_MODEL_PLUGIN(MecanumDriveController)
}
