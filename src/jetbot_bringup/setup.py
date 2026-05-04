from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'jetbot_bringup'

# `glob('maps/*')` also matches broken symlinks; setuptools then fails with
# "No such file or directory" when copying into install/share/.../maps/.
_map_files = sorted(
    p for p in glob('maps/*')
    if os.path.isfile(p)
)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/maps', _map_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='app',
    maintainer_email='obulichev@yandex.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_plan_costmap_debug_dump = jetbot_bringup.nav_plan_costmap_debug_dump:main',
            'robot_nav_bridge = jetbot_bringup.robot_nav_bridge:main',
            'motor_console_test = jetbot_bringup.motor_console_test:main',
            'scan_to_cloud = jetbot_bringup.scan_to_cloud:main',
        ],
    },
)
