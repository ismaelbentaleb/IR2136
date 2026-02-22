#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
    cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view', '--ros-args', '-r', 'image:=/annotated/image'],
    output='screen'
),
ExecuteProcess(
    cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view', '--ros-args', '-r', 'image:=/incidents/image'],
    output='screen'
),
    ])
