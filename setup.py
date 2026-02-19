# SPDX-FileCopyrightText: 2026 Michael Wimble <mike@wimblerobotics.com>
# SPDX-License-Identifier: Apache-2.0

from setuptools import setup

package_name = 'pi_gripper'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    include_package_data=True,  # Ensure this is True
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pi_gripper.launch.py']),
    ],
    install_requires=[
        'setuptools',
        'smbus2',  # Required for PCA9685 I2C communication
        'gpiod',   # Required for software PWM (GPIO control)
    ],
    zip_safe=True,
    maintainer='Michael Wimble',
    maintainer_email='mike@wimblerobotics.com',
    description='ROS2 node for controlling a servo gripper via PWM on Raspberry Pi 5.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pi_gripper_node = pi_gripper.pi_gripper_node:main',
        ],
    },
)
