from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'robot_common_launch'
maps_files = glob('config/maps/**/*.*', recursive=True)
maps_data_files = []
for file_path in maps_files:
    relative_dir = os.path.dirname(os.path.relpath(file_path, 'config/maps'))
    install_dir = os.path.join('share', package_name, 'config/maps', relative_dir)
    maps_data_files.append((install_dir, [file_path]))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/**/*.launch.py', recursive=True)),
        (os.path.join('share', package_name, 'config/rviz'), glob('config/rviz/*.rviz', recursive=True)),
        (os.path.join('share', package_name, 'config/nav2'), glob('config/nav2/*.*', recursive=True)),
        (os.path.join('share', package_name, 'config/cartographer'), glob('config/cartographer/*.*', recursive=True)),
        (os.path.join('share', package_name, 'config/gazebo'), glob('config/gazebo/*.*', recursive=True)),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/**/*.sdf', recursive=True)),
    ] + maps_data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Huang Zhenbiao',
    maintainer_email='biao876990970@hotmail.com',
    description='Common launch utilities for robots',
    license='BSD-3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
