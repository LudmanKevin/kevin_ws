import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    doosan_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('dsr_bringup2'),
                'launch',
                'dsr_bringup2_rviz.launch.py'
            )
        ),
        launch_arguments={
            'mode': 'real',
            'model': 'm1013',
            'host': '192.168.1.2'
        }.items()
    )

    fastapi_server = ExecuteProcess(
        cmd=['uvicorn', 'main:app', '--reload'],
        output='screen'
    )

    onrobot_eye_node = Node(
        package='onrobot_eye_integration',
        executable='onrobot_eye_node',
        name='onrobot_eye_node',
        output='screen'
    )

    onrobot_pickup_node = Node(
        package='onrobot_eye_integration',
        executable='onrobot_pickup',
        name='onrobot_pickup_node',
        output='screen'
    )

    delayed_nodes = TimerAction(
        period=5.0,
        actions=[
            fastapi_server,
            onrobot_eye_node,
            onrobot_pickup_node
        ]
    )

    # Bringup elindul elsőnek, többi 5s múlva
    return LaunchDescription([
        doosan_bringup,
        delayed_nodes
    ])