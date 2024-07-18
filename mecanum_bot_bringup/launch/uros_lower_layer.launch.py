from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():

    robot_description_launch_path = PathJoinSubstitution(
        [FindPackageShare('mecanum_bot_description'), 'launch', 'display.launch.py']
    )

    serial_port_id_arg = DeclareLaunchArgument(
        'serial_port_id',
        default_value='/dev/ttyUSB0',
        description='the serial port id for serial mirco-ros connection'
    )

    micro_ros_transport_arg = DeclareLaunchArgument(
        'micro_ros_transport',
        default_value='serial',
        description='micro-ROS transport protocol to use'
    )

    micro_ros_udp_port_arg = DeclareLaunchArgument(
        'micro_ros_port',
        default_value='8888',
        description='micro-ROS udp/tcp port number'
    )

    uros_baudate_arg = DeclareLaunchArgument(
        'uros_baudrate',
        default_value='921600',
        description='baudrate of uros'
    )


    micro_ros_agent_node_serial = Node(
        condition=LaunchConfigurationEquals('micro_ros_transport', 'serial'),
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', LaunchConfiguration("serial_port_id"), '--baudrate', LaunchConfiguration('uros_baudrate')]
    )

    mirco_ros_agent_node_wireless = Node(
        condition=LaunchConfigurationEquals('micro_ros_transport', 'udp4'),
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=[LaunchConfiguration('micro_ros_transport'), '--port', LaunchConfiguration('micro_ros_port')]
    )

    robot_desc_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_description_launch_path)
    )


    ld = LaunchDescription()

    ld.add_action(serial_port_id_arg)
    ld.add_action(uros_baudate_arg)
    ld.add_action(micro_ros_transport_arg)
    ld.add_action(micro_ros_udp_port_arg)
    ld.add_action(micro_ros_agent_node_serial)
    ld.add_action(mirco_ros_agent_node_wireless)
    ld.add_action(robot_desc_launcher)

    return ld
