from setuptools import setup

package_name = 'drone_mission'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/ardupilot_gazebo.launch.py']),
        ('share/' + package_name + '/scripts', [
            'scripts/auto_takeoff.py',
            'scripts/follower_gps.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='elena',
    maintainer_email='elena@todo.todo',
    description='ROS2 + PyMavLink drone mission',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'battery_gps_node1 = drone_mission.battery_gps_node1:main',
            'auto_takeoff = drone_mission.auto_takeoff:main',
        ],
    },
)

