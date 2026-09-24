"""
Interface web do operador (docs/architecture.md, seção 4.8).

Sobe três processos:
  - servidor HTTP estático da página (share/pantilt_web/web);
  - rosbridge_websocket (roslib.js <-> ROS 2);
  - web_video_server (stream MJPEG de /perception/debug_image).

Não sobe os nós de hardware, percepção ou controle: eles têm os próprios
launch files no pantilt_bringup ou são iniciados com ros2 run.

Uso:
  ros2 launch pantilt_web web.launch.py
  ros2 launch pantilt_web web.launch.py http_port:=8000 rosbridge_port:=9090 video_port:=8080

Se mudar rosbridge_port ou video_port, abra a página com ?ws=<porta>&video=<porta>.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    web_dir = os.path.join(get_package_share_directory('pantilt_web'), 'web')

    http_port = LaunchConfiguration('http_port')
    rosbridge_port = LaunchConfiguration('rosbridge_port')
    video_port = LaunchConfiguration('video_port')

    return LaunchDescription([
        DeclareLaunchArgument('http_port', default_value='8000',
                              description='Porta HTTP da página'),
        DeclareLaunchArgument('rosbridge_port', default_value='9090',
                              description='Porta WebSocket do rosbridge'),
        DeclareLaunchArgument('video_port', default_value='8080',
                              description='Porta HTTP do web_video_server'),

        ExecuteProcess(
            name='http_server',
            cmd=['python3', '-m', 'http.server', http_port, '--directory', web_dir],
            output='screen',
        ),

        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{
                'port': ParameterValue(rosbridge_port, value_type=int),
                # Um service ausente (ex.: inspection_manager ainda não iniciado)
                # não pode travar a ponte nem os outros comandos da página
                'call_services_in_new_thread': True,
                'default_call_service_timeout': 5.0,
            }],
        ),

        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen',
            parameters=[{
                'port': ParameterValue(video_port, value_type=int),
            }],
        ),
    ])
