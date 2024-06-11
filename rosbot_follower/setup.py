from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'rosbot_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='julia',
    maintainer_email='s4075758@student.rmit.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f"camera_track = {package_name}.camera_track:main",
            f"waypoint = {package_name}.waypoint:main",
            f"lidar_repeater = {package_name}.lidar_repeater:main",
            f"lidar_track = {package_name}.lidar_track:main",
            f"follow = {package_name}.follow:main",
            f"main_controller = {package_name}.main_controller:main",
            f"navigator = {package_name}.navigator:main",
            f"motion_controller = {package_name}.motion_controller:main",
        ],
    },
)
