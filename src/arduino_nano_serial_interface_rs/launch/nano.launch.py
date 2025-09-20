from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    package_dir = get_package_share_directory('arduino_nano_serial_interface_rs')
    config_file = os.path.join(package_dir, 'config', 'nano.yaml')
    return LaunchDescription([
        Node(
            package='arduino_nano_serial_interface_rs',
            executable='arduino_nano_serial_interface_node',
            name='arduino_nano_serial_interface_node',
            parameters=[config_file],
            output='log'
        )
    ])
