from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, RegisterEventHandler, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,OnProcessIO, OnProcessStart, OnShutdown)

import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # robot package directory 
    share_dir = get_package_share_directory('amr_ust_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'amr_ust.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()


    # remappings
    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static')
    ]
    
    # launch configurations
    namespace_1 = LaunchConfiguration('namespace_1')
    namespace_2 = LaunchConfiguration('namespace_2')

    # declaring launch arguments
    namespace_cmd = DeclareLaunchArgument(
        name='namespace_1',
        default_value='amr_bot',
        description='Giving top level namespace'
    )
    namespace_cmd_2 = DeclareLaunchArgument(
        name='namespace_2',
        default_value='amr_bot_2',
        description='Giving top level namespace'
    )

    # nodes to be launched on cmd
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace_1,
        parameters=[
            {'robot_description': robot_description}
        ],
        remappings = remappings,
    )
    robot_state_publisher_node_2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace_2,
        parameters=[
            {'robot_description': robot_description}
        ],
        remappings = remappings,
    )

    # gazebo server and client nodes
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
 
        launch_arguments={
            'pause': 'false',
        }.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace=namespace_1,
        arguments=[
            '-entity', 'amr_bot',
            '-topic', 'robot_description',
            '-robot_namespace', namespace_1,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
             
        ],
        output='screen',
    )

    spawn_entity_node_2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace=namespace_2,
        arguments=[
            '-entity', 'amr_bot_2',
            '-topic', 'robot_description',
            '-robot_namespace', namespace_2,
            '-x', '2.0',
            '-y', '0.0',
            '-z', '0.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
             
        ],
        output='screen',
    )


    return LaunchDescription([
        namespace_cmd,
        namespace_cmd_2,
        robot_state_publisher_node,
        robot_state_publisher_node_2,
        gazebo_server,
        gazebo_client,
        spawn_entity_node,
        spawn_entity_node_2,
 
    ])