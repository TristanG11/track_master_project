from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'track_master_real_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tristan',
    maintainer_email='tristanogandagaguerick@gmail.com',
    description='Package for launching different setup in real world',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

