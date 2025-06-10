from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tiago_dual_moveit_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='efallash',
    maintainer_email='efallashdez@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tiago_dual_moveit_py = tiago_dual_moveit_py.tiago_dual_moveit_py:main',
            'tiago_dual_commander = tiago_dual_moveit_py.tiago_dual_command_services:main',
            'workspace_test = tiago_dual_moveit_py.workspace_test:main',
            'pick_and_place = tiago_dual_moveit_py.tiago_dual_actions:pick_and_place',
            'tiago_dual_controller_monitor = tiago_dual_moveit_py.tiago_dual_controller_monitor:main',
            'tiago_demo = tiago_dual_moveit_py.tiago_dual_actions:main_demo',
        ],
    },
)
