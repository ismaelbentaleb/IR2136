from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='uav_mission',
            executable='flight_controller',
            output='screen'
        ),

        Node(
            package='uav_mission',
            executable='photo_trigger',
            output='screen'
        ),

        Node(
            package='uav_inspection',
            executable='photo_logger',
            output='screen'
        ),

        Node(
            package='uav_inspection',
            executable='incident_detector',
            output='screen'
        ),

        Node(
            package='uav_mission',
            executable='json_to_marker',
            output='screen'
        ),
    ])
