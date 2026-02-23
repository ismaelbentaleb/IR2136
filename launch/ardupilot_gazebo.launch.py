import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node

def generate_launch_description():

    # =====================
    # Rutas
    # =====================
    ardupilot_gazebo = os.path.expanduser('~/Documentos/GitHub/IR2136/ardupilot_gazebo')
    world_path = os.path.join(
        ardupilot_gazebo,
        'worlds',
        'neighbourhood.world'
    )

    rviz_config = os.path.expanduser(
        '~/Documentos/GitHub/IR2136/src/default_rviz.rviz'
    )

    # =====================
    # Gazebo
    # =====================
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            world_path
        ],
        output='screen'
    )

    # =====================
    # ArduPilot SITL (GAZEBO)
    # =====================
    leader_sitl = ExecuteProcess(
        cmd=[
            os.path.expanduser('~/ardupilot/Tools/autotest/sim_vehicle.py'),
            '-v', 'ArduCopter',
            '-f', 'gazebo-iris',
            '--console',
            '--map',
            '--out=udp:127.0.0.1:14550'
        ],
        output='screen'
    )

    # =====================
    # JSON -> Marker (ROS Node)
    # =====================
    json_to_marker = Node(
        package='uav_mission',
        executable='json_to_marker',
        name='json_to_marker',
        output='screen'
    )

    # =====================
    # RViz2
    # =====================
    rviz2 = ExecuteProcess(
        cmd=[
            'rviz2',
            '-d',
            rviz_config
        ],
        output='screen'
    )

    return LaunchDescription([

        # Variables de entorno necesarias
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=os.path.join(ardupilot_gazebo, 'models')
        ),

        # 🔥 IMPORTANTE: coherente con tu repo
        SetEnvironmentVariable(
            name='GAZEBO_PLUGIN_PATH',
            value=os.path.expanduser('~/ardupilot_gazebo/build/')
        ),

        # Orden correcto
        gazebo,

        # Esperar a que Gazebo levante bien
        TimerAction(
            period=10.0,
            actions=[leader_sitl]
        ),

        # Levantar marker y rviz después
        json_to_marker,
        
        TimerAction(
            period=13.0,
            actions=[rviz2]
        )
    ])


