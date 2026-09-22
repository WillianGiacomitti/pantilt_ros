# pantilt_ros

Workspace ROS 2 (Humble) do sistema de controle servo visual pan-tilt para orientação dinâmica de uma unidade de inspeção multimodal. Faz parte do TCC de Engenharia Mecatrônica da UTFPR.

A arquitetura completa (nós, tópicos, services, actions e máquina de estados) está em [`docs/architecture.md`](docs/architecture.md).

## Repositórios do projeto

| Repositório | Função |
|---|---|
| [pantilt_ros](https://github.com/WillianGiacomitti/pantilt_ros) | Nós ROS 2 (este) |
| [pantilt_dockerfile](https://github.com/WillianGiacomitti/pantilt_dockerfile) | Ambiente Docker |
| [pantilt_firmware](https://github.com/WillianGiacomitti/pantilt_firmware) | Firmware ESP32 |

## Uso (dentro do container)

```bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch pantilt_bringup system.launch.py
```

## Pacotes

| Pacote | Conteúdo |
|---|---|
| `pantilt_interfaces` | Mensagens, services e actions |
| `pantilt_hardware` | `serial_bridge_node`, `command_mux` |
| `pantilt_perception` | `camera_node`, `detector_node` |
| `pantilt_control` | `scan_node`, `visual_servo_node` (PID / fuzzy) |
| `pantilt_manager` | `inspection_manager` (máquina de estados) |
| `pantilt_web` | Interface web do operador |
| `pantilt_bringup` | Launch files e configuração |
