from setuptools import setup
import os
from glob import glob

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Installer le marker pour le package
        ('share/ament_index/resource_index/packages',
         [os.path.join('resource', package_name)]),
        # Installer le fichier package.xml
        (os.path.join('share', package_name), ['package.xml']),
        # Installer les fichiers de configuration YAML
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Installer les fichiers de lancement
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Package for controlling the differential drive robot using ROS 2 Control.',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
