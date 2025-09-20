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
    gnss_pkg = 'nmea_gnss_rs'
    nano_pkg = 'arduino_nano_serial_interface_rs'
    lidar_pkg = 'rplidar_ros'
    imu_pkg = 'bno055_imu'
    cam_pkg = 'camera_visualizer'
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

   # arduino interface 
    arduino_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(nano_pkg), 'launch', 'nano.launch.py'
            )
        )
    )

   # gnss pkg
    gnss_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(gnss_pkg), 'launch', 'nmea_gnss.launch.py'
            )
        )
    )

   # lidar launch 
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(lidar_pkg), 'launch', 'rplidar_c1_launch.py'
            )
        )
    )

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(imu_pkg), 'launch', 'imu.launch.py'
            )
        )
    )

    cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(cam_pkg), 'launch', 'camera_viz.launch.py'
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
        app_launch,
        gnss_launch,
        arduino_launch,
        lidar_launch,
        imu_launch,
        cam_launch
    ])
