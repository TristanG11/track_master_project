import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node  
def generate_launch_description():

    pkg_name_des = 'track_master'

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(pkg_name_des), 'launch', 'gazebo.launch.py'
            )
        )
    )

    return LaunchDescription([
        gazebo_launch,
    ])