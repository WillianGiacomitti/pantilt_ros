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
- Detecção: Ultralytics YOLO. Modelo do TCC em `/ros2_ws/models/equipment.pt` com o `equipment.yaml`; `yolo11n.pt` (COCO) com o `equipment_coco_test.yaml` só para teste. Classes identificadas por NOME (`model.names`), nunca por ID numérico.
- `numpy<2`, obrigatório para não quebrar o `cv_bridge` do Humble.

## Regras de trabalho

1. **Planeje antes de editar.** Liste os arquivos que vai criar ou modificar e espere aprovação.
2. **Escopo estrito.** Não edite arquivos fora do escopo da tarefa pedida, nem para "melhorar" algo que viu de passagem. Anote a sugestão na resposta.
3. **Não faça commits** nem push. O autor revisa o diff e commita.
4. **Não adicione dependências** (apt, pip, package.xml) sem perguntar.
5. **Uma tarefa = um nó ou uma funcionalidade.** Não implemente vários nós de uma vez.
6. Comentários, docstrings e mensagens de log em **português**.
7. Unidades SI nos tópicos (rad, rad/s). Graus só com sufixo `_deg` (architecture.md §2, princípio 6).
8. Todo parâmetro é declarado com `declare_parameter` e tem valor padrão documentado no `params.yaml`.
9. Lógica pura fora dos nós, em módulos com pytest (ex.: `scan_pattern.py`, `state_machine.py`).
10. Ao terminar, explique o que foi feito arquivo por arquivo e como testar manualmente (comandos `ros2 run`, `ros2 topic echo`, `ros2 action send_goal`).

## Comandos (dentro do container)

```bash
ws                                   # alias: cd /ros2_ws + source do ROS e do install/
colcon build --symlink-install && ws

# sistema completo (docs/testes.md §6.2); hardware:=false, web:=false, recorder:=false desligam partes
ros2 launch pantilt_bringup system.launch.py                 # inspeção
ros2 launch pantilt_bringup system.launch.py mode:=dataset   # coleta (o mesmo que o dataset.launch.py)

# por camada
ros2 launch pantilt_bringup hardware.launch.py      # serial_bridge_node + command_mux
ros2 launch pantilt_bringup perception.launch.py    # camera_node + detector_node
ros2 launch pantilt_bringup control.launch.py       # scan_node + visual_servo_node
ros2 launch pantilt_web web.launch.py               # página (8080), rosbridge (9090), web_video_server (8081)
ros2 run pantilt_manager inspection_manager --ros-args \
  --params-file /ros2_ws/src/pantilt_ros/pantilt_bringup/config/params.yaml

# ensaios (o system.launch.py já sobe o experiment_recorder)
ros2 service call /experiment/configure pantilt_interfaces/srv/ConfigureExperiment \
  "{label: 'degrau_kp0.8', notes: '', image_mode: 'none'}"   # gravações em /ros2_ws/ensaios/<rótulo>/
ros2 run pantilt_experiment analisar_ensaios                 # métricas e figuras em /ros2_ws/ensaios/_analise/

# testes da lógica pura
python3 -m pytest pantilt_control/test pantilt_manager/test   # (a partir de /ros2_ws/src/pantilt_ros)
```

## Estrutura

```
pantilt_interfaces/   msg, srv, action (ament_cmake)
pantilt_hardware/     serial_bridge_node, command_mux (+ command_arbiter.py, limits.py, serial_protocol.py)
pantilt_perception/   camera_node, detector_node, calibration_node
                      (+ camera_source.py, camera_info_file.py, detection.py, calibration_math.py)
pantilt_control/      scan_node, visual_servo_node (+ scan_pattern.py, visual_servo_math.py,
                      controllers/{base,pid,fuzzy}.py)
pantilt_manager/      inspection_manager (+ state_machine.py)
pantilt_web/          web/index.html + js/ + launch (http, rosbridge, web_video_server)
pantilt_bringup/      launch/{system,hardware,perception,control,dataset}.launch.py,
                      config/{params.yaml, equipment.yaml, equipment_coco_test.yaml,
                      camera_intrinsics.yaml, fastdds.xml}
pantilt_dataset/      capture_node (auxiliar: vídeos para o dataset)
pantilt_experiment/   experiment_recorder (auxiliar: ensaios) + session.py, csv_writer.py,
                      snapshot_writer.py; análise: analysis.py, plots.py, analisar_ensaios.py
docs/                 architecture.md (fonte da verdade), testes.md (roteiros de teste),
                      ajustes_pantilt_dockerfile.md, SerialProtocol.h (cópia do protocolo do firmware)
```

`calibration_node`, `capture_node` e `experiment_recorder` são auxiliares ao desenvolvimento e ficam fora do diagrama da arquitetura.

## Estado atual (23/10/2026)

Todos os nós da arquitetura estão implementados e testados no hardware: hardware, percepção, calibração, `scan_node`, `visual_servo_node` (PID; fuzzy só com pytest), `inspection_manager`, web, `system.launch.py`, coleta de dataset, `experiment_recorder` (com imagens e bloco Ensaio na página) e `analisar_ensaios`.

Pendências:

- [ ] **Tarefa E: sinais, sintonia e ensaios** (em espera; o autor sintoniza com o sistema completo). Roteiro em `docs/testes.md` §4.2:
  1. com `kp = 0.2` e `ki = kd = 0`, confirmar o sentido de giro e registrar `invert_pan`/`invert_tilt` na §4.2 (hoje `false`/`true` no `params.yaml`, já centralizou na inspeção), com a mão no botão de parada;
  2. subir `kp` até 0,8–1,0 e observar sobressinal e oscilação;
  3. ativar `ki` só no ensaio com alvo em movimento;
  4. registrar em `docs/testes.md` §4.2: sinais, CENTER, TRACK, alvo perdido, cancelamento, prioridade do operador e limite de ângulo;
  5. gravar os ensaios com o `experiment_recorder`, um rótulo por configuração de ganhos (§7.2).
- [ ] Sinais de diagnóstico e `experiment_recorder` no hardware (`docs/testes.md` §7.1 e §7.2; testados só com planta simulada).
- [x] `controllers/fuzzy.py` (Sugeno de ordem zero, architecture.md §4.5), com pytest em `test_fuzzy.py`; falta o teste funcional com `controller: fuzzy` (goal `Center` direto: log `Controlador fuzzy`, P/I/D zerados no `/control/servo_debug`). A partir daqui o código congela: só entram correções que impeçam um ensaio de rodar.
- [ ] Remover do `serial_protocol.py` (e do `SerialProtocol.h`) o legado de encoder e auto home quando o firmware for limpo (architecture.md §8).
- [ ] `.gitignore` não cobre os `.pytest_cache` dos pacotes.

Atualize esta lista ao concluir cada item (o autor confirma).

## Decisões de projeto JÁ FECHADAS (não reabrir)

Fundamentação em `docs/architecture.md` (§4.3, §4.5, §4.10) e na seção teórica da monografia.

1. **Entrada do controlador: erro ANGULAR**, `θ = atan(e_px / f)`, não erro em pixel. O fator `(1+x²)` da matriz de interação é a derivada da tangente: a planta fica linear e os ganhos independem de resolução e lente.
2. **Saída: velocidade angular** (rad/s) em `/ptu/cmd_vel_auto`.
3. **Dois controladores SISO** independentes (pan ← erro horizontal, tilt ← vertical). Sem inversão de matriz; a geometria vira `invert_pan`/`invert_tilt`.
4. **Ganhos:** `K_p` [s⁻¹] (≡ λ do IBVS; teto teórico ~3 s⁻¹ com 165 ms de atraso), `K_i` [s⁻²], `K_d` [adimensional].
5. **`f_x` e `f_y` vêm de `/camera/camera_info`**, nunca como parâmetro nem constante.
6. **Publicação desacoplada:** servo a 20 Hz repetindo o último comando por até `cmd_hold_s` (detector ~10 Hz, watchdog do mux 0,3 s).
7. **Métricas:** `t_c` da primeira detecção válida até o erro entrar e permanecer na tolerância por `hold_time_s`; `e_r` é a média nessa janela.
8. **Calibração por rotação:** `c_x`, `c_y` fixos no centro geométrico; Levenberg-Marquardt em numpy (sem scipy); só comandos de posição; não volta ao *home* se o operador assumir ou com Ctrl+C.
9. **Fuzzy:** Sugeno de ordem zero, 5×3 = 15 regras, sem ação integral e sem parâmetro ROS novo. `kp` é o fator de escala (`erro_max = max_output/kp`): com tabela linear, o fuzzy é exatamente o P. A tabela de regras só muda se os ensaios mostrarem necessidade.
10. **Readquisição:** só a `Center` abortada com o texto `alvo perdido` (`LOST_MESSAGE` em `state_machine.py`) leva a nova varredura; STOP e qualquer jog da página abortam a inspeção.

## Armadilhas conhecidas

- **Telemetria = contagem de passos:** não há encoder nem auto home. A posição em `/joint_states` é a contagem de passos do firmware, relativa ao zero manual (`/ptu/set_zero` com o mecanismo no centro mecânico). Um passo perdido desloca a posição sem aviso, e os limites do bridge e do firmware, o `scan_node` e a calibração confiam nela. Um eixo travado mecanicamente não aparece em `/joint_states`. Antes de malha fechada, confira com jog curto (`docs/testes.md` §1.2).
- **Webcam no WSL2:** com o kernel WSL 6.18 o UVC funciona: a câmera USB (anexada com `usbipd`) aparece como `/dev/video0` e entrega 640×480 MJPG a até 30 fps. Use `source: "0"`. Sem UVC, o `camera_node` aceita URL (stream MJPEG do Windows).
- **Imagens via DDS:** o SHM padrão do Fast DDS (512 KB) não comporta 640×480 (921 KB). Todo processo ROS precisa de `FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/src/pantilt_ros/pantilt_bringup/config/fastdds.xml` e o container de `/dev/shm` bem maior que 64 MB (`docs/ajustes_pantilt_dockerfile.md`). O `web_video_server` só recebe tópicos BEST_EFFORT com `qos_profile=sensor_data` na URL, e o tópico vai com `/` literal (não decodifica `%2F`).
- **Portas da web:** o docker-compose publica só 8080, 9090 e 8081. Outra porta funciona dentro do container, mas não chega ao navegador do Windows.
- **Heartbeat serial:** o fail-safe do firmware é de 500 ms; o heartbeat do bridge deve ser 5 Hz, nunca 2 Hz.
- **Protocolo serial:** definido em `pantilt_firmware/include/Serialprotocol.h` (cópia em `docs/SerialProtocol.h`). Não altere tipos ou payloads sem alterar o firmware.
- **rosbridge + actions:** a web não usa actions; usa os services `/inspection/*` e o tópico `/inspection/status`.
- **Pesos `.pt`** não vão para o git. Ficam em `/ros2_ws/models/`; o `detector_node` não baixa nada e encerra se `model_path` não existir (download do `yolo11n.pt` em `docs/testes.md` §2.2).
- **Detector só em CPU:** sem CUDA e com ~3,7 GB de RAM no WSL. A `imgsz` 640, ~10 quadros/s e ~115 ms de atraso (inferência: 640 = 73 ms, 480 = 46 ms, 320 = 32 ms). Os quadros que chegam durante a inferência são descartados de propósito.
- **`f_x` depende da resolução:** mudar a resolução do `camera_node` ou trocar de câmera (OAK-D) invalida a calibração.
- **Atraso da imagem em rotinas que movem os eixos:** espere o eixo parar E descarte os alvos com `header.stamp` anterior à parada.
- **Ctrl+C em nós com `SignalHandlerOptions.NO`:** o `KeyboardInterrupt` só chega quando a thread principal volta ao Python. Use laço com `spin_once(timeout_sec=0.1)`, como no `scan_node`.
- **Futures fora dos callback groups:** no rclpy do Humble, os callbacks de `add_done_callback` rodam como tasks do executor, sem respeitar os callback groups. Com `MultiThreadedExecutor`, proteja o estado com um lock, como no `inspection_manager`.
- **CLI do ROS lenta:** no volume 9p, `ros2 topic echo`/`hz` leva vários segundos para começar. Para medir, prefira um script `rclpy` ou espere mais.
- **Disco:** `/ros2_ws` é o `C:` do Windows, com pouco espaço. Nunca grave imagens cruas com `ros2 bag` por longos períodos (~20 MB/s); coleta e ensaios usam MP4.
