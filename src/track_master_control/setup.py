from setuptools import setup
import os
from glob import glob

package_name = 'track_master_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tristan Agoumbi',
    maintainer_email='tristanogandagaguerick@gmail.com',
    description='Package for controlling the differential drive robot using ROS 2 Control.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)

