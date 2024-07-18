import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, PythonExpression, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch_ros.actions import Node


def generate_launch_description():

    realsense_cam_frame_arg = DeclareLaunchArgument(
        'realsense_frame_id',
        default_value='camera_link',
        description='A sample realsense camera frame id specifer'
    )


    realsense_camera_launch_path = PathJoinSubstitution(
        [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py']
    )

    realsense_camera_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_camera_launch_path),
        launch_arguments={
            'camera_namespace': '',
            'rgb_camera.color_profile': '640,480,30',
            'enable_sync': 'true',
            'enable_depth': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '1',
            'depth_module.depth_profile': '640,480,30',
            'pointcloud.enable': 'true',
            'ordered_pc': 'true', 
            'initial_reset': 'true',
        }.items()   
    )

    fake_laser_config_path = PathJoinSubstitution(
        [FindPackageShare('mecanum_bot_bringup'), 'config', 'fake_laser_setup.yaml']
    )


    fake_lase_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        remappings=[('depth', 'camera/depth/image_rect_raw'),
                    ('depth_camera_info', 'camera/depth/camera_info'),
                    ('scan', 'cam_scan')],
        parameters=[fake_laser_config_path]
    )


    ld = LaunchDescription()

    ld.add_action(realsense_camera_launcher)
    ld.add_action(fake_lase_node)

    return ld