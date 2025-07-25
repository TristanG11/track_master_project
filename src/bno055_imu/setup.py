from setuptools import find_packages, setup

package_name = 'bno055_imu'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tristan',
    maintainer_email='tristanogandagaguerick@gmail.com',
    description='ROS 2 node to publish BNO055 IMU data and temperature.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bno055_imu_node = bno055_imu.bno055_imu_node:main',
        ],
    },
)
