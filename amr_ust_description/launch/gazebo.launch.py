from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('amr_ust_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'amr_ust.xacro')
    robot_description_config = xacro.process_file(xacro_file) 
    robot_urdf = robot_description_config.toxml()

    world_file = 'small_warehouse3.world'
    world_file_path = os.path.join(share_dir, 'worlds', world_file)

    world = LaunchConfiguration('world')

    world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_file_path,
        description='Full path to the world model file to load'
    )

    pause_sim_cmd = DeclareLaunchArgument(
        'pause_sim',
        default_value='true',
        description='whether to pause or play the simulation when gazebo env is launced'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='whether to use simulation time or real-sys time'
    )

    rqt_configuration_arg = DeclareLaunchArgument(
        'use_config_tool',
        default_value='false',
        description='whether to start the rqt configuration tool for dynamic param configuration'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf, 'use_sim_time': LaunchConfiguration('use_sim_time')} # 'use_sim_time': LaunchConfiguration('use_sim_time')
        ]
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'pause': LaunchConfiguration('pause_sim'),
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

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'amr_ust',
            '-topic', 'robot_description',
            '-z', '0.05'
        ],
        output='screen'
    )

    rqt_reconfigure_node = Node(
        condition=IfCondition(LaunchConfiguration('use_config_tool')),
        package='rqt_reconfigure',
        executable='rqt_reconfigure',
        name='rqt_reconfigure',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],     
    )

    dynamic_obstacles_actor = Node(
        package='dynamic_obstacles',
        executable='pallet_jack_actor',
        name='pallet_jack_actor',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    return LaunchDescription([
        world_cmd,
        pause_sim_cmd,
        use_sim_time_arg,
        rqt_configuration_arg,
        robot_state_publisher_node,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
        # dynamic_obstacles_actor,
        rqt_reconfigure_node
    ])
 