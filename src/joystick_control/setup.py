from setuptools import find_packages, setup

package_name = 'joystick_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/joystick_params.yaml']),
        ('share/'+ package_name + '/launch',['launch/joystick_control.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tristan Agoumbi',
    maintainer_email='tristanogandagaguerick@gmail.com',
    description='Control the robot speed using a PS4 controller',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'joystick_control_node = joystick_control.joystick_control_node:main'
        ],
    },
)


