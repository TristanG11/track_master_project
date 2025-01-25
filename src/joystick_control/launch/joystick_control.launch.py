from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('joystick_control')

    config_file = os.path.join(package_dir, 'config', 'joystick_params.yaml')

    return LaunchDescription([
        Node(
            package= 'joystick_control',
            executable='joystick_control_node',
            name='joystick_control_node',
            parameters=[config_file]
        )
    ])
