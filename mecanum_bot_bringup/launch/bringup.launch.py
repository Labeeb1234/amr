from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # description_launch_path = PathJoinSubstitution(
    #     [FindPackageShare('mecanum_bot_description'), 'launch', 'display.launch.py']
    # )

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("mecanum_bot_ekf"), "config", "ekf_params.yaml"]
    )

    micro_ros_launch_path = PathJoinSubstitution(
        [FindPackageShare('mecanum_bot_bringup'), 'launch', 'uros_lower_layer.launch.py']
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

    micro_ros_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(micro_ros_launch_path)
    )



    ld = LaunchDescription()
    ld.add_action(robot_localization_node)
    ld.add_action(micro_ros_launcher)


    

    return ld
