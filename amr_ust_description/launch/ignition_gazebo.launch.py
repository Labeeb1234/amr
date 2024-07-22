from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def safe_load_urdf_file(file_path):
    try:
        with open(file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        return None

def generate_launch_description():
    share_dir = get_package_share_directory('amr_ust_description')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # robot description file
    urdf_file = 'amr_ust_new.urdf'
    print(f"loaded urdf: {urdf_file}")
    robot_urdf_fpath = os.path.join(share_dir, 'urdf/', urdf_file)
    robot_desc = safe_load_urdf_file(robot_urdf_fpath)


    # world files and file path
    world_file = 'new_world.world'
    world_file_path = os.path.join(share_dir, 'worlds/', world_file)

    # ros-ignition bridge node param file path
    bridge_params = os.path.join(share_dir,'config/', 'amr_ust_bridge_param.yaml')



    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='whether to use gazebo simulation time or not'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': use_sim_time
            }]
    )

    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher'
    # )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r -s -v4 {}'.format(world_file_path),
            'on_exit_shutdown': 'true'
        }.items()
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4 '}.items()
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],

    )

    urdf_spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'amr_ust',
            '-file', robot_urdf_fpath,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        output='screen'

    )


    ld = LaunchDescription()
    ld.add_action(use_sim_time_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(urdf_spawn_node)
    ld.add_action(gz_ros2_bridge)
    
    

    return ld