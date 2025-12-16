from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtlebot_nav_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')), # Include launch files in build
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')), # Include world files in build
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')), # Include world files in build
        (os.path.join('share', package_name, 'config'), glob('config/*')), # Include world files in build
        (os.path.join('share', package_name, 'maps'), glob('maps/*')), # Include world files in build

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='maintainer@email.generic',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simple_navigator = turtlebot_nav_control.simple_navigator:main',
            'map_visualizer = turtlebot_nav_control.map_visualizer:main'
        ],
    },
)
