# SPDX-FileCopyrightText: 2026 Michael Wimble <mike@wimblerobotics.com>
# SPDX-License-Identifier: Apache-2.0

from setuptools import setup
from setuptools.command.develop import develop as _develop


# colcon-core's PythonBuildTask (used by colcon-ros for ament_python) invokes
#   python setup.py develop --editable --build-directory <path> --no-deps
# and later:
#   python setup.py develop --uninstall --editable --build-directory <path>
#
# setuptools removed --editable and --build-directory from the develop command.
# Re-add them as no-ops so that `colcon build --symlink-install` keeps working
# with modern setuptools without requiring any changes to the colcon toolchain.
class develop(_develop):
    user_options = _develop.user_options + [
        ('editable', None,
         'No-op: develop is already editable by definition.'),
        ('build-directory=', None,
         'No-op: colcon manages the build directory itself.'),
    ]

    def initialize_options(self):
        super().initialize_options()
        self.editable = None
        self.build_directory = None

    def finalize_options(self):
        super().finalize_options()


package_name = 'pi_gripper'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pi_gripper.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michael Wimble',
    maintainer_email='mike@wimblerobotics.com',
    description='ROS2 node for controlling a servo gripper via PWM on Raspberry Pi 5.',
    license='Apache-2.0',
    cmdclass={'develop': develop},
    entry_points={
        'console_scripts': [
            'pi_gripper_node = pi_gripper.pi_gripper_node:main',
        ],
    },
)
