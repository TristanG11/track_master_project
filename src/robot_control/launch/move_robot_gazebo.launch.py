import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    pkg_name_ctrl = 'robot_control'


    robot_control_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(pkg_name_ctrl), 'launch', 'robot_control_gazebo.launch.py'
            )
        )
    )

    joystick_control_node = Node(
            package='joystick_control',
            executable='joystick_control_node',
            output='screen',
            remappings=[
                ('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')
            ]
        )

    return LaunchDescription([
        robot_control_gazebo_launch,
        joystick_control_node,
    ])
