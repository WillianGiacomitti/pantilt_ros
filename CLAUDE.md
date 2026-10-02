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
ws                                   # alias: cd /ros2_ws + source do ROS e do install/
colcon build --symlink-install && ws

# testes por camada (launch files em pantilt_bringup)
ros2 launch pantilt_bringup hardware.launch.py      # serial_bridge_node + command_mux
ros2 launch pantilt_bringup control.launch.py       # scan_node
ros2 launch pantilt_web web.launch.py               # página (8080), rosbridge (9090), web_video_server (8081)
ros2 launch pantilt_bringup dataset.launch.py       # coleta: câmera + capture_node (+ hardware, controle e web)
ros2 launch pantilt_bringup perception.launch.py    # camera_node + detector_node (pesos em /ros2_ws/models)
ros2 launch pantilt_bringup system.launch.py        # (a criar)

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
pantilt_dataset/      capture_node (ferramenta auxiliar: vídeos para o dataset)
docs/                 architecture.md (fonte da verdade), testes.md (roteiros de teste por pacote),
                      ajustes_pantilt_dockerfile.md, diagnostico_encoders.md (histórico),
                      SerialProtocol.h (cópia do protocolo do firmware)
```

## Estado atual

- [X] `pantilt_interfaces`: definições criadas (validar com `colcon build`)
- [X] `pantilt_hardware`: migrar `serial_bridge_node.py` do repo `pantilt_dockerfile`
- [x] `pantilt_hardware`: `command_mux`
- [x] `pantilt_web`: migrar `index.html` e adicionar vídeo, seleção e status
- [x] `pantilt_perception`: `camera_node`
- [x] `pantilt_perception`: `detector_node`
- [x] `pantilt_web`: testar com a camera (`docs/testes.md` §2 e §3)
- [ ] `pantilt_control`: `visual_servo_node` (PID)
- [x] `pantilt_control`: `scan_node`: testado com juntas simuladas e no hardware (29/09/2026); melhorias de `docs/diagnostico_encoders.md` §4 pendentes
- [x] `pantilt_interfaces`: `StartCapture.srv` e `CaptureStatus.msg`
- [X] `pantilt_dataset`: `capture_node`
- [x] `pantilt_web`: painel de coleta de dataset e `dataset.launch.py`: testado no hardware (02/10/2026)
- [ ] `pantilt_manager`: `inspection_manager`
- [ ] `pantilt_web`: testar com a inspecao
- [ ] `pantilt_bringup`: launch files e config (feitos: `hardware.launch.py`, `control.launch.py`, `dataset.launch.py`, `perception.launch.py`; falta `system`)
- [ ] `pantilt_control`: controlador fuzzy

Atualize esta lista ao concluir cada item (o autor confirma).

## Ponto atual e próximos passos (02/10/2026)

**Feito:** percepção completa e testada no hardware:
- `camera_node` e `detector_node` (`perception.launch.py`);
- `yolo11n` em CPU com o `equipment_coco_test.yaml`.

Medidas de referência do detector (`docs/testes.md` §2.2):
- **imgsz 640:** ~10 quadros/s e ~115 ms de atraso desde a captura;
- **inferência pura:** 640 = 73 ms, 480 = 46 ms e 320 = 32 ms.

A coleta do dataset (`docs/testes.md` §5.4) segue em paralelo, com o autor.

**Próximo passo: `pantilt_control/visual_servo_node` com PID** (architecture.md §4.5; roteiro da §12, item 5). É o servidor da action `/control/center`:
- converte o erro em px de `/perception/target` em velocidade de pan e tilt;
- publica em `/ptu/cmd_vel_auto`, passando pelo `command_mux`.

Ele é testado sem o manager, com `ros2 action send_goal`. O filtro é posto antes pela CLI (`/perception/set_target`), porque sem filtro o detector não publica alvo. Esse teste já mede *t_c* e *e_r* (`Center.Result`, Quadro 3 do TCC).

O que já existe para reaproveitar:
- **`scan_node`:** o padrão de action server (goal novo substitui o anterior, cancelamento, zero ao terminar, `MultiThreadedExecutor` com `spin_once`) e a separação da lógica pura (`scan_pattern.py`) para testar com pytest;
- **`Center.action`:**
  - goal: `mode`, `tolerance_px`, `hold_time_s`, `lost_timeout_s`;
  - feedback: `error_x/y/px`, `target_visible`, `centered`;
  - result: `final_error_px` e `convergence_time_s`;
- **`VisualTarget`:**
  - `error_x > 0` = alvo à direita, `error_y > 0` = alvo abaixo;
  - o header é o da imagem;
  - `detected=false` quando o alvo some.

**Decisões a tomar no planejamento** (perguntar ao autor; a arquitetura é vaga nestes pontos):
1. **Unidade do erro no PID:**
   - em px, com ganhos em (rad/s)/px;
   - ou convertido em ângulo pelo campo de visão (`atan(erro / f_px)`), com ganhos em 1/s, independentes da resolução e do `imgsz`. Exige medir `f_px`, ou o FOV, e cria um parâmetro novo na §4.5.

   Sugestão de medida: jog de pan de 10° e medir o deslocamento de um objeto em px.
2. **Sinais:** o sentido da imagem com pan e tilt positivos (`docs/testes.md` §3, testes 5 e 7) define `invert_pan`/`invert_tilt`. Confirmar no hardware antes do primeiro goal, com velocidade baixa.
3. **Taxa e atraso:** o laço roda a cada alvo (~10 Hz, ~115 ms de atraso).
   - Os ganhos iniciais devem ser baixos.
   - O `command_mux` zera a velocidade após 0,3 s sem comando. Se o detector cair abaixo de ~3 Hz, o eixo para entre os quadros. Decidir se o servo republica o último comando ou se isso fica como limitação.
4. **Zona morta e derivada:** a bbox oscila alguns px entre quadros.
   - Usar zona morta dentro de `tolerance_px`?
   - Derivada sobre a medida, com filtro?
   - Anti-windup do integral na saturação (`max_vel_deg_s`)?
5. **Alvo perdido e limites:**
   - Aborta se não chegar nenhum `/perception/target` (filtro não definido) em `lost_timeout_s`?
   - E se o eixo encostar no limite do bridge sem convergir? Hoje o servo empurraria para sempre.
6. **Métrica *t_c*:** contar do início do goal ou da primeira detecção válida (a §10 diz "da primeira detecção válida até a estabilização")? E *e_r*: média dos últimos quadros dentro do `hold_time_s`, ou o último erro?
7. **Parâmetros novos** (ex.: `f_px`, zona morta), a registrar na §4.5 **antes** do código.

**Entregas esperadas:**
- `controllers/pid.py` (puro, com pytest), com a interface `compute(erro, dt) -> velocidade` prevista para o fuzzy;
- `visual_servo_node.py`;
- `visual_servo_node` no `control.launch.py` e no `params.yaml`;
- seção em `docs/testes.md`:
  - sinais;
  - CENTER e TRACK;
  - alvo perdido;
  - cancelamento;
  - prioridade do operador;
  - registro com `ros2 bag record /perception/target /ptu/cmd_vel_auto /joint_states` para os gráficos do TCC.

**Depois disso**, na ordem da §12:
1. `inspection_manager` (§4.3 e §6), que liga detector, scan e servo;
2. teste da inspeção pela página;
3. `system.launch.py`;
4. controlador fuzzy e comparação com o PID;
5. scripts de ensaio (rosbag → métricas).

**Pendências sem prazo:**
- melhorias do `scan_node` em `docs/diagnostico_encoders.md` §4;
- comentário do `ListEquipment.srv`: ainda diz "cruzado com as classes do modelo". Ajustar na tarefa do `inspection_manager`, que lista só o yaml (architecture.md §4.3, v0.3).

## Armadilhas conhecidas

- **Webcam no WSL2:** no ambiente atual (kernel WSL 6.18) o driver UVC funciona: a câmera USB, anexada com `usbipd`, aparece como `/dev/video0` no container e entrega 640×480 MJPG a até 30 fps (menos com pouca luz, por causa da exposição automática). Use `source: "0"`. Em kernels sem UVC, o `camera_node` também aceita URL como `source` (stream MJPEG do Windows).
- **Imagens via DDS:** o SHM padrão do Fast DDS (512 KB) não comporta uma imagem 640×480 (921 KB). Sem ajuste, os quadros vão por UDP e se perdem em BEST_EFFORT. Todo processo ROS precisa de `FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/src/pantilt_ros/pantilt_bringup/config/fastdds.xml` e o container precisa de `/dev/shm` bem maior que 64 MB (ver `docs/ajustes_pantilt_dockerfile.md`). O `web_video_server` só recebe tópicos BEST_EFFORT com `qos_profile=sensor_data` na URL do stream, e não decodifica `%2F`: o tópico vai na URL com `/` literal.
- **Portas da web:** o docker-compose do `pantilt_dockerfile` publica só 8080 (página), 9090 (rosbridge) e 8081 (web_video_server). Esses são os padrões do `web.launch.py` e do `config.js`. Uma porta fora dessa lista funciona dentro do container (`curl`), mas não chega ao navegador do Windows.
- **Heartbeat serial:** o fail-safe do firmware é de 500 ms. O heartbeat do bridge deve ser de 5 Hz, nunca 2 Hz.
- **Protocolo serial:** definido em `pantilt_firmware/include/Serialprotocol.h`. Não altere tipos ou payloads sem alterar o firmware.
- **rosbridge + actions:** a web não usa actions diretamente; usa os services `/inspection/*` e o tópico `/inspection/status`.
- **Pesos `.pt`** não vão para o git (ver `.gitignore`). Ficam em `/ros2_ws/models/`; o `detector_node` não baixa nada (comando de download em `docs/testes.md` §2.2).
- **Detector só em CPU:** o container não tem CUDA e o WSL tem ~3,7 GB de RAM. O `detector_node` processa ~10 quadros/s a `imgsz` 640 e descarta de propósito os quadros que chegam durante a inferência. Toda malha que depende de `/perception/target` roda nessa taxa, com ~115 ms de atraso.
- **Telemetria = encoders:** a posição em `/joint_states` vem dos encoders AS5600, não da contagem de passos. Um encoder ruim produz um ângulo falso e plausível, e os limites do bridge e o `scan_node` confiam nele. Antes de qualquer malha fechada, confira `/joint_states` com jog curto (`docs/testes.md` §1.2).
- **Ctrl+C em nós com `SignalHandlerOptions.NO`:** o `KeyboardInterrupt` só chega quando a thread principal volta ao Python. Um `executor.spin()` sem timer fica bloqueado em C, e o nó não sai. Use laço com `spin_once(timeout_sec=0.1)`, como no `scan_node`.
- **CLI do ROS lenta:** no volume 9p, um `ros2 topic echo`/`hz` leva vários segundos para começar a receber. Timeouts curtos dão falsa impressão de tópico mudo; para medir, prefira um script `rclpy` ou espere mais.
- **Coleta de dataset e disco:** `/ros2_ws` é o `C:` do Windows, com pouco espaço livre. Nunca grave imagens cruas com `ros2 bag` por longos períodos (~20 MB/s); a coleta usa MP4.
