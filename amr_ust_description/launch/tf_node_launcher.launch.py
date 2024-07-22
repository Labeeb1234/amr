from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    odom_to_baseTF_node = Node(
        package='basic_tf',
        executable='odom_tf_node',
        name='odom_to_baseTF'
    )
 
    base_to_wheelTF_node = Node(
        package='wheel_tf',
        executable='wheel_tf_node',
        name="base_to_wheeTF"
    )

    wheel_to_rollerTF_node = Node(
        package='rollers_tf',
        executable='roller_tf_node',
        name='wheel_to_rollerTF'
    )

    return LaunchDescription([
        # odom_to_baseTF_node,
        base_to_wheelTF_node,
        wheel_to_rollerTF_node

    ])
