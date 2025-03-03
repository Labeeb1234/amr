import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction, LogInfo, DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

from launch.event_handlers import OnProcessExit
import xacro


def generate_launch_description():

    dynamic_obstacle_arg = DeclareLaunchArgument(
        'dynamic_obs',
        default_value='false',
        description='whether to start moving obstacles or not'
    )

    nav_commander_node = Node(
        package='amr_ust_navigation',
        executable='nav_commander.py',
        name='nav_commander',
        output='both',
    )
    
    # pallet-jack controller
    pallet_jack_controller = Node(
        condition=IfCondition(LaunchConfiguration('dynamic_obs')),
        package='dynamic_obstacles',
        executable='pallet_jack_actor',
        name='pallet_jack_actor',
        output='screen'
    )
    
    ld = LaunchDescription()
    ld.add_action(dynamic_obstacle_arg)
    ld.add_action(nav_commander_node)
    ld.add_action(pallet_jack_controller)

    return ld