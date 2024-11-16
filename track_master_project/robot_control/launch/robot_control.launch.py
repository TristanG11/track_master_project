from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('robot_control'), 'config', 'diff_drive_controller.yaml'
    )
    urdf_path = os.path.join(
        get_package_share_directory('track_master'), 'urdf', 'tracker_master_bot.urdf.xacro'
    )

    return LaunchDescription([

        # Spawner du joint_state_broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        ),
        
        # Spawner du diff_drive_controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller'],
            output='screen'
        ),
    ])
