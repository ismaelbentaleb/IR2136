import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


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

    return LaunchDescription([

        # Variables de entorno necesarias
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=os.path.join(ardupilot_gazebo, 'models')
        ),

        SetEnvironmentVariable(
    		name='GAZEBO_PLUGIN_PATH',
    		value=os.path.expanduser('~/ardupilot_gazebo/build/')
		),


        # Orden correcto
        gazebo,

        TimerAction(
            period=5.0,
            actions=[leader_sitl]
        )
    ])

