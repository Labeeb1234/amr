from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node


def generate_launch_description():
    
    lidar_frame_id_arg = DeclareLaunchArgument(
        'laser_frame_id',
        default_value='laser',
        description='to specify the lidar frame id name'
    )

    lidar_serial_port_arg = DeclareLaunchArgument(
        'lidar_serial_port',
        default_value='/dev/ttyUSB1',
        description='specify lidar serial port id'
    )

    use_laser_filter = DeclareLaunchArgument(
        'use_laser_filter',
        default_value='false',
        description='whether to use laser filter or not'
    )

    rplidar_a1_launch_path = PathJoinSubstitution(
        [FindPackageShare('rplidar_ros'), 'launch', 'rplidar_a1_launch.py']
    )

    rplidar_a2m8_launch_path = PathJoinSubstitution(
        [FindPackageShare('rplidar_ros'), 'launch', 'rplidar_a2m8_launch.py']
    )

    A1rplidar_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_a1_launch_path),
        launch_arguments={
            'serial_port': LaunchConfiguration('lidar_serial_port'), 
            'frame_id': LaunchConfiguration('laser_frame_id')
        }.items()
    )

    A2rplidar_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_a2m8_launch_path),
        launch_arguments={
            'serial_port': LaunchConfiguration('lidar_serial_port'),
            'frame_id': LaunchConfiguration('laser_frame_id')
        }.items()
    )

    # laser filter
    laser_filter_config_path = PathJoinSubstitution(
        [FindPackageShare('mecanum_bot_bringup'), 'config', 'box_laser_filter.yaml']
    )

    scan_to_scan_filter_node = Node(
        condition=LaunchConfigurationEquals('use_laser_filter', 'true'),
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[
            laser_filter_config_path
        ]
    )



    ld = LaunchDescription()

    ld.add_action(lidar_frame_id_arg)
    ld.add_action(lidar_serial_port_arg)
    ld.add_action(use_laser_filter)
    # ld.add_action(A1rplidar_launcher)
    ld.add_action(scan_to_scan_filter_node)
    ld.add_action(A2rplidar_launcher)


    return ld