import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_name_ctrl = 'track_master_control'
    pkg_websocket = 'rosbridge_server'

    # Inclure le launch du contrôle du robot
    move_robot_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(pkg_name_ctrl), 'launch', 'move_robot_real.launch.py'
            )
        )
    )

    # Inclure le launch du serveur websocket (format XML)
    app_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(pkg_websocket), 'launch', 'rosbridge_websocket_launch.xml'
            )
        )
    )

    # Ajouter un délai de 5 secondes avant de lancer le websocket
    delayed_app_launch = TimerAction(
        period=5.0,
        actions=[app_launch]
    )

    return LaunchDescription([
        move_robot_real,
        delayed_app_launch
    ])
