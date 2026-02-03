from setuptools import find_packages, setup

package_name = 'turtlebot3_waypoint_navigator'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RoboSapiens',
    maintainer_email='contact@robosapiens.org',
    description='Autonomous waypoint-based navigation for TurtleBot 3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_navigator_nav2=turtlebot3_waypoint_navigator.navigator_nav2:main',
            'waypoint_navigator_twist=turtlebot3_waypoint_navigator.navigator_twist:main',
        ],
    },
)
