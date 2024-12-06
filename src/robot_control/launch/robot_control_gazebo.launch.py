import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node  
def generate_launch_description():

    pkg_name_des = 'track_master'
    pkg_name_ctrl = 'robot_control'

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(pkg_name_des), 'launch', 'gazebo.launch.py'
            )
        )
    )

    robot_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(pkg_name_ctrl), 'launch', 'robot_control.launch.py'
            )
        )
    )

    delayed_robot_control_launch = TimerAction(
        period=10.0,
        actions=[robot_control_launch]
    )

    map_frame_publisher_node = Node(
        package='frame_manager',
        executable='map_frame_publisher',
        name='map_frame_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True}  # Set use_sim_time to True if simulation time is used
        ]
    )

   
    return LaunchDescription([
        gazebo_launch,
        delayed_robot_control_launch,
        map_frame_publisher_node
    ])