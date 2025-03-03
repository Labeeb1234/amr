import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
import xacro

def generate_launch_description():
    world_name = os.getenv('WORLD')
    world = world_name + '.world'
    world_file_path = os.path.join(
        get_package_share_directory('amr_ust_description'),
        'worlds',
        world
    )
    
    # Get the urdf file
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    model_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )

    urdf_model = 'turtlebot3_'+ TURTLEBOT3_MODEL + '.urdf'
    model_urdf_xacro = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'urdf',
        urdf_model
    )
    print(f"loading urdf: {model_urdf_xacro}")
    robot_description_config = xacro.process_file(model_urdf_xacro) 
    robot_desc = robot_description_config.toxml()

    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # Declare the launch arguments
    use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='whether to use simulation time or not'
    )
    
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify namespace of the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify namespace of the robot')

    pause_sim_cmd = DeclareLaunchArgument(
        'pause_sim',
        default_value='true',
        description='whether to pause simulator on launch or not'
    )

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', TURTLEBOT3_MODEL,
            '-file', model_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.05'
        ],
        output='screen',
    )

    # launching gazebo
    gazebo_classic_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'pause': LaunchConfiguration('pause_sim'),
            'world': world_file_path
        }.items()
    )

    gazebo_classic_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    # getting the BASE tf states of the bot using robot state publisher  
    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': robot_desc
            }],
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_cmd)
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(pause_sim_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(gazebo_classic_server)
    ld.add_action(gazebo_classic_client)
    ld.add_action(urdf_spawn_node)
    
    return ld
