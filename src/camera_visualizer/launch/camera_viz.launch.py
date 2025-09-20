from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_visualizer',           # 
            executable='camera_visualizer_node',
            name='camera_visualizer_node',              
            output='log'                    
        )
    ])
