from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    # Récupérer le chemin du répertoire du package
    package_dir = get_package_share_directory('nmea_gnss_cpp')

    # Construire le chemin complet vers le fichier YAML
    config_file = os.path.join(package_dir, 'config', 'nmea_gnss_params.yaml')

    return LaunchDescription([
        Node(
            package='nmea_gnss_cpp',
            executable='nmea_gnss_node',
            name='nmea_gnss_node',
            parameters=[config_file]
        )
    ])
