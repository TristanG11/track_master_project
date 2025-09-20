from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bno055_imu',           # Nom de ton package
            executable='bno055_imu_node',# Ton nœud Python
            name='bno055_imu',              # Nom du nœud dans ROS
            output='log'                    # Logs uniquement dans ~/.ros/log
        )
    ])
