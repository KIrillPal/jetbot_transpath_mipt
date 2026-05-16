from setuptools import find_packages, setup

package_name = 'jetbot_grid_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='app',
    maintainer_email='obulichev@yandex.ru',
    description='Python grid global planner for Nav2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'python_grid_planner = jetbot_grid_planner.python_grid_planner:main',
        ],
    },
)
