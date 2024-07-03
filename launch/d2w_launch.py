from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    ld = LaunchDescription()
    share_dir = get_package_share_directory('d2w_ros2')
    parameter_file = LaunchConfiguration('params_file')

    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'comms.yaml'),
                                           description='Path to the ROS2 parameters file to use.')
    comms_node = Node(
        package='d2w_ros2',
        executable='comms',
        name='comms_node',
        output="screen",
        emulate_tty=True,
        parameters=[parameter_file]
    )

    vel_node = Node(
        package='d2w_ros2',
        executable='vel_drive.py',
        name='vel_drive',
        output="screen"
    )

    ref_node = Node(
        package='d2w_ros2',
        executable='ref_publisher.py',
        output="screen"
    )

    plot_node = Node(
        package='d2w_ros2',
        executable='plot.py',
        output="screen"
    )

    control_node = Node(
        package='d2w_ros2',
        executable='control_drive.py',
        output="screen"
    )

    ld.add_action(plot_node)
    ld.add_action(params_declare)
    ld.add_action(comms_node)
    ld.add_action(vel_node)
    ld.add_action(ref_node)
    ld.add_action(control_node)
    return ld
