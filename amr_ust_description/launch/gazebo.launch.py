from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('amr_ust_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'amr_ust.xacro')
    robot_description_config = xacro.process_file(xacro_file) 
    robot_urdf = robot_description_config.toxml()

    world_file = 'sample_world.world'
    world_file_path = os.path.join(share_dir, 'worlds', world_file)



    world = LaunchConfiguration('world')

    world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_file_path,
        description='Full path to the world model file to load'
    )

    pause_sim_cmd = DeclareLaunchArgument(
        'pause_sim',
        default_value='false',
        description='whether to pause or play the simulation when gazebo env is launced'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='whether to use simulation time or real-sys time'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf, 'use_sim_time': LaunchConfiguration('use_sim_time')}
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
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    rviz2_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('amr_ust_description'), 'launch', 'display.launch.py'])
        )
    )

    # controller manager nodes
    # mecanumbot_controller_spawner = Node( 
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['mecanumbot_controller', '--controller-manager', '/controller_manager']
    # )

    # joint_broad_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_broad', '--controller-manager', '/controller_manager']
    # )


    return LaunchDescription([
        world_cmd,
        pause_sim_cmd,
        use_sim_time_arg,
        robot_state_publisher_node,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
        # rviz2_launcher
        # joint_broad_spawner,
        # mecanumbot_controller_spawner
    ])
 