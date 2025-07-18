import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import AnyLaunchDescriptionSource

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
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(pkg_websocket), 'launch', 'rosbridge_websocket_launch.xml'
            )
        )
    )

    # Env var needs to be set
    # Lancer l'app npm
    # npm_app_path = os.environ.get('NPM_APP_PATH')
    # if not npm_app_path:
    #     raise RuntimeError("La variable d'environnement NPM_APP_PATH n'est pas définie")

    # #npm_app_path = "/home/tristan/ros2_ws/src/robot_project/src/track_master_app"
    # npm_start_process = ExecuteProcess(
    #     cmd=["npm", "start"],
    #     cwd=npm_app_path,
    #     shell=True,
    #     output='screen'
    # )

    # # Lancer le websocket + npm app après 5s
    # delayed_app_launch = TimerAction(
    #     period=5.0,
    #     actions=[app_launch, npm_start_process]
    # )

    return LaunchDescription([
        move_robot_real,
        app_launch
    ])
