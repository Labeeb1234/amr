from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share_dir = get_package_share_directory('amr_ust_description')

    world_file = 'cup.world'
    world_file_path = os.path.join(share_dir, 'worlds', world_file)


    # launch configurations (use if required --> add in return space)
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    use_simulator = LaunchConfiguration('use_simulator')
    use_world = LaunchConfiguration('use_world')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # declaring launch arguments
    headless_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient'
    )

    world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_file_path,
        description='Full path to the world model file to load'
    )

    use_simulator_cmd = DeclareLaunchArgument(
        name='use_simulator',
        default_value='True',
        description='whether to use gazebo simulator or not'
    )

    use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False',
        description='whether to use gazebo simulation time or not'
    )


    # nodes/servers and clients to launch to launch
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
            'world': world
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

    return LaunchDescription([
        world_cmd,
        gazebo_server,
        gazebo_client,
    ])