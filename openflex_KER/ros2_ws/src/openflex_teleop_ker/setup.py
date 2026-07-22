#!/usr/bin/env python3
from glob import glob

from setuptools import find_packages, setup

package_name = 'openflex_teleop_ker'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chengdu Changshu Robot Co., Ltd',
    maintainer_email='openarmrobot@gmail.com',
    description='OpenArm KER USB, serial, and WiFi teleoperation adapter for OpenFlex.',
    license='OpenArmX Research and Education License',
    entry_points={
        'console_scripts': [
            'ker_driver_node = openflex_teleop_ker.ker_driver_node:main',
            'ker_arm_bridge_node = openflex_teleop_ker.arm_bridge_node:main',
        ],
    },
)
