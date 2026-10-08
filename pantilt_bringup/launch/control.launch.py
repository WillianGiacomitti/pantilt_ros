"""
Camada de controle (docs/architecture.md, seções 4.4 e 4.5).

Sobe os nós do pantilt_control com os parâmetros do params.yaml:
  - scan_node (varredura em zigue-zague, action /control/scan).
O visual_servo_node entra aqui quando for implementado.

Os comandos saem em /ptu/cmd_vel_auto e só chegam ao pan-tilt pelo
command_mux: suba também o hardware.launch.py.

Uso:
  ros2 launch pantilt_bringup control.launch.py
  ros2 launch pantilt_bringup control.launch.py params_file:=/caminho/params.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('pantilt_bringup'), 'config', 'params.yaml')

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Arquivo de parâmetros dos nós'),

        Node(
            package='pantilt_control',
            executable='scan_node',
            name='scan_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
