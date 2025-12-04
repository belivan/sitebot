from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sitebot_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro') + glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools', 'roslibpy'],
    zip_safe=True,
    maintainer='SiteBot Developer',
    maintainer_email='user@example.com',
    description='ROS2 package for SiteBot with Gazebo Harmonic and Nav2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mujoco_bridge = sitebot_ros.mujoco_bridge:main',
            'lidar_publisher = sitebot_ros.lidar_publisher:main',
        ],
    },
)
