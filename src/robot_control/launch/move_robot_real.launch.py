import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    pkg_name_ctrl = 'robot_control'

    robot_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(pkg_name_ctrl), 'launch', 'real_robot_control.launch.py'
            )
        )
    )

    joystick_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('joystick_control'), 'launch', 'joystick_control.launch.py'
            )
        )
    )


    return LaunchDescription([
        robot_control_launch,
        joystick_control_launch,
    ])
