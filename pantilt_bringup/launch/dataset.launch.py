"""
Coleta de vídeos para o dataset da YOLO (docs/architecture.md, seção 4.9).

Sobe, com os parâmetros do params.yaml:
  - camera_node (pantilt_perception), sem o detector: a coleta não precisa da YOLO;
  - capture_node (pantilt_dataset), services /capture/* e /capture/status;
  - com hardware:=true, o hardware.launch.py e o control.launch.py
    (bridge, command_mux e scan_node: jog e varredura durante a gravação);
  - com web:=true, o web.launch.py do pantilt_web (página com o painel de coleta).

Uso:
  ros2 launch pantilt_bringup dataset.launch.py
  ros2 launch pantilt_bringup dataset.launch.py hardware:=false web:=false
  ros2 launch pantilt_bringup dataset.launch.py params_file:=/caminho/params.yaml

Página: http://localhost:8080/?tab=coleta
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('pantilt_bringup')
    web_dir = get_package_share_directory('pantilt_web')
    default_params = os.path.join(bringup_dir, 'config', 'params.yaml')

    params_file = LaunchConfiguration('params_file')
    hardware = LaunchConfiguration('hardware')
    web = LaunchConfiguration('web')

    def include(path, condition, arguments=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(path),
            launch_arguments=(arguments or {}).items(),
            condition=IfCondition(condition),
        )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Arquivo de parâmetros dos nós'),
        DeclareLaunchArgument('hardware', default_value='true',
                              description='Sobe hardware.launch.py e control.launch.py '
                                          '(jog e varredura)'),
        DeclareLaunchArgument('web', default_value='true',
                              description='Sobe o web.launch.py (página, rosbridge e '
                                          'web_video_server)'),

        Node(
            package='pantilt_perception',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='pantilt_dataset',
            executable='capture_node',
            name='capture_node',
            output='screen',
            parameters=[params_file],
        ),

        include(os.path.join(bringup_dir, 'launch', 'hardware.launch.py'), hardware,
                {'params_file': params_file}),
        include(os.path.join(bringup_dir, 'launch', 'control.launch.py'), hardware,
                {'params_file': params_file}),
        include(os.path.join(web_dir, 'launch', 'web.launch.py'), web),
    ])
