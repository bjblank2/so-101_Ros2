import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'so_arm_feetech_interface'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Brian Blankenau',
    maintainer_email='brianjblank7@gmail.com',
    description='Hardware interface for the so-101 arm with FeeTech servos.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'feetech_interface_node = so_arm_feetech_interface.feetech_interface_node:main',
            'leader_follower_teleop_node = so_arm_feetech_interface.leader_follower_teleop_node:main',
            'leader_arm_node = so_arm_feetech_interface.leader_arm_node:main',
            'follower_arm_node = so_arm_feetech_interface.follower_arm_node:main',
            'calibration_node = so_arm_feetech_interface.calibration_node:main',
            'calibration_client = so_arm_feetech_interface.calibration_client:main',
            'test_leader_follower = so_arm_feetech_interface.test_leader_follower:main',
        ],
    },
) 