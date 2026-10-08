"""
Camada de acionamento (docs/architecture.md, seções 4.6 e 4.7).

Sobe os dois nós do pantilt_hardware com os parâmetros do params.yaml:
  - serial_bridge_node (ponte serial com a ESP32);
  - command_mux (prioridade do operador e watchdog de comando).

Uso:
  ros2 launch pantilt_bringup hardware.launch.py
  ros2 launch pantilt_bringup hardware.launch.py params_file:=/caminho/params.yaml

A porta serial e os limites ficam só no params.yaml (serial_bridge_node).
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
            package='pantilt_hardware',
            executable='serial_bridge_node',
            name='serial_bridge_node',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='pantilt_hardware',
            executable='command_mux',
            name='command_mux',
            output='screen',
            parameters=[params_file],
        ),
    ])
