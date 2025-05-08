import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled

    pkg_name_des='track_master'
    pkg_name_ctrl = 'track_master_control'

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(pkg_name_des),'launch','rsp_real.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    map_frame_publisher_node = Node(
        package='frame_manager',
        executable='map_frame_publisher',
        name='map_frame_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': False}  # Set use_sim_time to True if simulation time is used
        ]
    )

    param_file = os.path.join(
    get_package_share_directory('track_master_controller'),
    'config',
    'controller_config.yaml'
    )

    track_master_controller_node = Node(
    package='track_master_controller',
    executable='track_master_controller_node',
    name='track_master_controller',
    output='screen',
    parameters=[param_file]
    )

    # Launch them all!
    return LaunchDescription([
        rsp,
        #delayed_controller_manager,
        #delayed_diff_drive_spawner,
        #delayed_joint_broad_spawner,
        map_frame_publisher_node,
        track_master_controller_node
    ])



"""     controller_params_file = os.path.join(get_package_share_directory(pkg_name_ctrl),'config','diff_drive_controller.yaml')  #to comment 

    controller_manager = Node(   #to comment 
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )

    delayed_controller_manager = TimerAction(period=15.0, actions=[controller_manager]) #to comment 

    diff_drive_spawner = Node(   #to comment 
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler( #to comment 
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(   #to comment 
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    ) 

    delayed_joint_broad_spawner = RegisterEventHandler(  #to comment 
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )"""

    