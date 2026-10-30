import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'f450_brain'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Naman Harshwal',
    maintainer_email='namanharshwal@github.com',
    description='F450 Quadcopter Autonomous Flight Control and Verification Package for PX4 2.4.8 and ROS 2 Humble',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'commander = f450_brain.commander:main',
            'verify_motors = f450_brain.verify_motors:main',
            'direct_sequence = f450_brain.direct_sequence:main',
            'visualizer = f450_brain.visualizer:main',
            'f450_sim = f450_brain.f450_sim:main',
            'keyboard_teleop = f450_brain.keyboard_teleop:main',
        ],
    },
)
