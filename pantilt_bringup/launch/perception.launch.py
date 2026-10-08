"""
Camada de percepção (docs/architecture.md, seções 4.1 e 4.2).

Sobe os nós do pantilt_perception com os parâmetros do params.yaml:
  - camera_node (publica /camera/image_raw);
  - detector_node (YOLO: /perception/detections, /perception/target,
    /perception/debug_image e o service /perception/set_target).

Os pesos ficam em /ros2_ws/models (model_path no params.yaml). O detector não
baixa pesos: veja docs/testes.md, seção 2.2.

Uso:
  ros2 launch pantilt_bringup perception.launch.py
  ros2 launch pantilt_bringup perception.launch.py params_file:=/caminho/params.yaml
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
            package='pantilt_perception',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='pantilt_perception',
            executable='detector_node',
            name='detector_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
