# CLAUDE.md — pantilt_ros

Instruções para o Claude Code neste repositório. Leia este arquivo inteiro antes de qualquer tarefa.

## Contexto

TCC de Engenharia Mecatrônica (UTFPR): sistema de controle servo visual (IBVS) para um mecanismo pan-tilt que orienta uma unidade de inspeção multimodal de linhas de transmissão. Este repositório é o workspace ROS 2 (conteúdo de `/ros2_ws/src`).

O projeto tem três repositórios:

- `pantilt_ros` (este): nós ROS 2;
- `pantilt_dockerfile`: ambiente Docker;
- `pantilt_firmware`: firmware da ESP32.

**Fonte da verdade:** `docs/architecture.md`. Nomes de nós, tópicos, services, actions, mensagens e parâmetros DEVEM seguir esse documento. Se uma tarefa exigir mudar um contrato, pare e proponha a mudança no documento antes de alterar o código.

## Stack

- ROS 2 Humble, rodando em Docker (osrf/ros:humble-desktop) sobre Windows/WSL2.
- Tudo em Python (rclpy). A única exceção é `pantilt_interfaces`, que é ament_cmake por exigência do rosidl.
- Detecção: Ultralytics YOLO (`yolo11n`). Classes identificadas por NOME (`model.names`), nunca por ID numérico.
- `numpy<2`, obrigatório para não quebrar o `cv_bridge` do Humble.

## Regras de trabalho

1. **Planeje antes de editar.** Liste os arquivos que vai criar ou modificar e espere aprovação.
2. **Escopo estrito.** Não edite arquivos fora do escopo da tarefa pedida, nem para "melhorar" algo que viu de passagem. Anote a sugestão na resposta.
3. **Não faça commits** nem push. O autor revisa o diff e commita.
4. **Não adicione dependências** (apt, pip, package.xml) sem perguntar.
5. **Uma tarefa = um nó ou uma funcionalidade.** Não implemente vários nós de uma vez.
6. Comentários, docstrings e mensagens de log em **português**.
7. Unidades SI nos tópicos (rad, rad/s). Graus só em parâmetros com sufixo `_deg`.
8. Todo parâmetro é declarado com `declare_parameter` e tem valor padrão documentado no `params.yaml`.
9. Ao terminar, explique o que foi feito arquivo por arquivo e como testar manualmente (comandos `ros2 run`, `ros2 topic echo`, `ros2 action send_goal`).

## Comandos (dentro do container)

```bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash

# testes por camada (launch files em pantilt_bringup)
ros2 launch pantilt_bringup hardware.launch.py
ros2 launch pantilt_bringup perception.launch.py
ros2 launch pantilt_bringup control.launch.py
ros2 launch pantilt_bringup system.launch.py

# conferir interfaces
ros2 interface show pantilt_interfaces/action/Center
```

## Estrutura planejada

```
pantilt_interfaces/   msg, srv, action (ament_cmake)
pantilt_hardware/     serial_bridge_node, command_mux
pantilt_perception/   camera_node, detector_node
pantilt_control/      scan_node, visual_servo_node, controllers/{pid,fuzzy}.py
pantilt_manager/      inspection_manager, state_machine.py
pantilt_web/          web/index.html + launch (http, rosbridge, web_video_server)
pantilt_bringup/      launch/, config/{params.yaml, equipment.yaml, equipment_coco_test.yaml}
docs/                 architecture.md
```

## Estado atual

- [X] `pantilt_interfaces`: definições criadas (validar com `colcon build`)
- [X] `pantilt_hardware`: migrar `serial_bridge_node.py` do repo `pantilt_dockerfile`
- [x] `pantilt_hardware`: `command_mux`
- [x] `pantilt_web`: migrar `index.html` e adicionar vídeo, seleção e status
- [x] `pantilt_perception`: `camera_node`
- [ ] `pantilt_perception`: `detector_node`
- [ ] `pantilt_web`: testar com a camera
- [ ] `pantilt_control`: `visual_servo_node` (PID)
- [ ] `pantilt_control`: `scan_node`
- [ ] `pantilt_manager`: `inspection_manager`
- [ ] `pantilt_web`: testar com a inspecao
- [ ] `pantilt_bringup`: launch files e config
- [ ] `pantilt_control`: controlador fuzzy

Atualize esta lista ao concluir cada item (o autor confirma).

## Armadilhas conhecidas

- **Webcam no WSL2:** no ambiente atual (kernel WSL 6.18) o driver UVC funciona: a câmera USB, anexada com `usbipd`, aparece como `/dev/video0` no container e entrega 640×480 MJPG a até 30 fps (menos com pouca luz, por causa da exposição automática). Use `source: "0"`. Em kernels sem UVC, o `camera_node` também aceita URL como `source` (stream MJPEG do Windows).
- **Imagens via DDS:** o SHM padrão do Fast DDS (512 KB) não comporta uma imagem 640×480 (921 KB). Sem ajuste, os quadros vão por UDP e se perdem em BEST_EFFORT. Todo processo ROS precisa de `FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/src/pantilt_ros/pantilt_bringup/config/fastdds.xml` e o container precisa de `/dev/shm` bem maior que 64 MB (ver `docs/ajustes_pantilt_dockerfile.md`). O `web_video_server` só recebe tópicos BEST_EFFORT com `qos_profile=sensor_data` na URL do stream, e não decodifica `%2F`: o tópico vai na URL com `/` literal.
- **Heartbeat serial:** o fail-safe do firmware é de 500 ms. O heartbeat do bridge deve ser de 5 Hz, nunca 2 Hz.
- **Protocolo serial:** definido em `pantilt_firmware/include/Serialprotocol.h`. Não altere tipos ou payloads sem alterar o firmware.
- **rosbridge + actions:** a web não usa actions diretamente; usa os services `/inspection/*` e o tópico `/inspection/status`.
- **Pesos `.pt`** não vão para o git (ver `.gitignore`).
