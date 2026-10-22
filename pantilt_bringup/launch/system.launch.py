"""
Sistema completo do pan-tilt em um comando (docs/architecture.md, seção 3).

Dois modos, escolhidos pelo argumento mode:

  mode:=inspection (padrão): inspeção autônoma
    - hardware.launch.py (serial_bridge_node + command_mux), se hardware:=true;
    - perception.launch.py (camera_node + detector_node);
    - control.launch.py (scan_node + visual_servo_node);
    - inspection_manager (pantilt_manager);
    - experiment_recorder (pantilt_experiment), se recorder:=true: grava cada
      inspeção em /ros2_ws/ensaios (docs/architecture.md, seção 4.11);
    - web.launch.py do pantilt_web, se web:=true.
    Página: http://localhost:8080/

  mode:=dataset: coleta de vídeos para o dataset da YOLO
    - o dataset.launch.py, com os mesmos hardware e web (câmera, capture_node e,
      com hardware:=true, bridge, command_mux e scan_node). Sem a YOLO.
    Página: http://localhost:8080/?tab=coleta

Com hardware:=false, o controle continua de pé na inspeção: os goals são aceitos e
dá para exercitar o gerenciador e a página sem a ESP32, mas os comandos não chegam
ao eixo.

Uso:
  ros2 launch pantilt_bringup system.launch.py
  ros2 launch pantilt_bringup system.launch.py mode:=dataset
  ros2 launch pantilt_bringup system.launch.py hardware:=false web:=false
  ros2 launch pantilt_bringup system.launch.py recorder:=false
  ros2 launch pantilt_bringup system.launch.py params_file:=/caminho/params.yaml

Não suba junto com outro launch que tenha câmera (dataset.launch.py,
perception.launch.py): dois camera_node disputam o /dev/video0.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition, LaunchConfigurationEquals
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
    recorder = LaunchConfiguration('recorder')

    def include(path, arguments=None, condition=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(path),
            launch_arguments=(arguments or {}).items(),
            condition=condition,
        )

    def bringup_launch(name):
        return os.path.join(bringup_dir, 'launch', name)

    params = {'params_file': params_file}
    inspection = LaunchConfigurationEquals('mode', 'inspection')
    dataset = LaunchConfigurationEquals('mode', 'dataset')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Arquivo de parâmetros dos nós'),
        DeclareLaunchArgument('mode', default_value='inspection',
                              choices=['inspection', 'dataset'],
                              description='inspection: inspeção autônoma (YOLO, servo e '
                                          'gerenciador); dataset: coleta de vídeos'),
        DeclareLaunchArgument('hardware', default_value='true',
                              description='Sobe o hardware.launch.py (bridge e command_mux)'),
        DeclareLaunchArgument('web', default_value='true',
                              description='Sobe o web.launch.py (página, rosbridge e '
                                          'web_video_server)'),
        DeclareLaunchArgument('recorder', default_value='true',
                              description='Sobe o experiment_recorder (só no modo inspection)'),

        LogInfo(condition=inspection,
                msg='Sistema em modo inspeção. Página: http://localhost:8080/'),
        LogInfo(condition=dataset,
                msg='Sistema em modo coleta. Página: http://localhost:8080/?tab=coleta'),

        GroupAction(condition=inspection, actions=[
            include(bringup_launch('hardware.launch.py'), params, IfCondition(hardware)),
            include(bringup_launch('perception.launch.py'), params),
            include(bringup_launch('control.launch.py'), params),

            Node(
                package='pantilt_manager',
                executable='inspection_manager',
                name='inspection_manager',
                output='screen',
                parameters=[params_file],
            ),

            Node(
                package='pantilt_experiment',
                executable='experiment_recorder',
                name='experiment_recorder',
                output='screen',
                parameters=[params_file],
                condition=IfCondition(recorder),
            ),

            include(os.path.join(web_dir, 'launch', 'web.launch.py'),
                    condition=IfCondition(web)),
        ]),

        include(bringup_launch('dataset.launch.py'),
                {'params_file': params_file, 'hardware': hardware, 'web': web},
                dataset),
    ])
