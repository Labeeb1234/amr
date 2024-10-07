from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("mecanum_bot_ekf"), "config", "alt_ekf_params.yaml"]
    )

    lidar_launch_path = PathJoinSubstitution(
        [FindPackageShare('mecanum_bot_bringup'), 'launch', 'laser.launch.py']
    )

    cam_or_depth_launch_path = PathJoinSubstitution(
        [FindPackageShare('mecanum_bot_bringup'), 'launch', 'cam_or_depth.launch.py'] 
    )

    micro_ros_launch_path = PathJoinSubstitution(
        [FindPackageShare('mecanum_bot_bringup'), 'launch', 'uros_lower_layer.launch.py']
    )

    mecanum_bot_description_launch_path = PathJoinSubstitution(
        [FindPackageShare('mecanum_bot_description'), 'launch', 'display.launch.py']
    )


    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_path
        ]
    )

    rqt_reconfigure = Node(
        package='rqt_reconfigure',
        executable='rqt_reconfigure',
        name='rqt_reconfigure',
        output='screen',
    )

    reset_handler_node = Node(
        package='gui_tools',
        executable='mcu_reset_handler.py',
        name='reset_handler',
        output='screen'
    )

    imu_handler_node = Node(
        package='imu_handler',
        executable='imu_handler',
        name='imu_handler',
        output='screen'
    )

    mecanum_bot_description_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mecanum_bot_description_launch_path),
        launch_arguments={
            'show_display_rviz': 'false',
        }.items()
    )

    lidar_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_path)
    )

    micro_ros_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(micro_ros_launch_path)
    )

    realsense_cam_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cam_or_depth_launch_path)
    )



    ld = LaunchDescription()
    ld.add_action(mecanum_bot_description_launcher)
    ld.add_action(robot_localization_node)
    ld.add_action(lidar_launcher)
    # ld.add_action(realsense_cam_launcher)
    ld.add_action(micro_ros_launcher)
    ld.add_action(reset_handler_node)
    ld.add_action(imu_handler_node)
    ld.add_action(rqt_reconfigure)
    

    return ld
