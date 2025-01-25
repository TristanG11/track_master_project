import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
def generate_launch_description():
    pkg_name_ctrl = 'track_master_control'

    move_robot_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(pkg_name_ctrl), 'launch', 'move_robot_gazebo.launch.py'
            )
        )
    )
    websocket_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml'
            )
        )
    )
  
    npm_start_command = ExecuteProcess(
        cmd=['npm', 'start'],
        cwd= '/home/tristan/ros2_ws/src/robot_project/src/track_master_app',
        name='npm_start'
    )

    return LaunchDescription([
        move_robot_gazebo,
        websocket_launch,
        npm_start_command,
        
    ])
