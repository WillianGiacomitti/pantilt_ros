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
ros2 launch pantilt_bringup control.launch.py       # scan_node (+ visual_servo_node)
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
pantilt_perception/   camera_node, detector_node, calibration_node
pantilt_control/      scan_node, visual_servo_node, controllers/{base,pid,fuzzy}.py
pantilt_manager/      inspection_manager, state_machine.py
pantilt_web/          web/index.html + launch (http, rosbridge, web_video_server)
pantilt_bringup/      launch/, config/{params.yaml, equipment.yaml, equipment_coco_test.yaml,
                      camera_intrinsics.yaml}
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
- [x] `pantilt_control`: `scan_node`: testado com juntas simuladas e no hardware (29/09/2026); melhorias de `docs/diagnostico_encoders.md` §4 pendentes
- [x] `pantilt_interfaces`: `StartCapture.srv` e `CaptureStatus.msg`
- [X] `pantilt_dataset`: `capture_node`
- [x] `pantilt_web`: painel de coleta de dataset e `dataset.launch.py`: testado no hardware (02/10/2026)
- [ ] **`pantilt_perception`: `calibration_node` + `CameraInfo` no `camera_node`** (tarefa A, em andamento)
- [ ] **`pantilt_hardware`: `/ptu/cmd_pos_auto` no `command_mux`** (pré-requisito da tarefa A)
- [ ] **medir `f_x` e `f_y` no hardware** e conferir contra o FOV (tarefa B)
- [ ] `pantilt_control`: `controllers/{base,pid}.py` com pytest (tarefa C)
- [ ] `pantilt_control`: `visual_servo_node` (tarefa D)
- [ ] `pantilt_control`: teste de sinais e sintonia no hardware (tarefa E)
- [ ] `pantilt_manager`: `inspection_manager`
- [ ] `pantilt_web`: testar com a inspecao
- [ ] `pantilt_bringup`: launch files e config (feitos: `hardware.launch.py`, `control.launch.py`, `dataset.launch.py`, `perception.launch.py`; falta `system`)
- [ ] `pantilt_control`: `controllers/fuzzy.py` (depois; a interface já deve estar pronta)

Atualize esta lista ao concluir cada item (o autor confirma).

## Decisões de projeto JÁ FECHADAS (não reabrir)

Fundamentação completa em `docs/architecture.md` §4.5 e na seção teórica da monografia.

1. **Entrada do controlador: erro ANGULAR**, não erro em pixel. O nó converte
   `θ = atan(e_px / f)` antes do controlador. Motivo: o fator `(1+x²)` da matriz de
   interação é exatamente a derivada da tangente, então a mudança de variável lineariza
   a planta de forma exata e torna os ganhos independentes de resolução e lente.
2. **Saída: velocidade angular** (rad/s) em `/ptu/cmd_vel_auto`.
3. **Dois controladores SISO** independentes: pan recebe o erro horizontal, tilt o vertical.
   Não há inversão de matriz no código; a geometria vira `invert_pan` / `invert_tilt`.
4. **Unidades dos ganhos:** `K_p` [s⁻¹], `K_i` [s⁻²], `K_d` [adimensional].
   `K_p` equivale ao ganho λ do IBVS. Teto teórico ~3 s⁻¹ (margem de fase de 60° com
   165 ms de atraso); começar entre 0,5 e 1,0.
5. **`f_x` e `f_y` vêm de `/camera/camera_info`**, publicado pelo `camera_node`.
   Nunca como parâmetro do `visual_servo_node` nem como constante no código.
6. **Publicação desacoplada:** o servo publica a 20 Hz repetindo o último comando por
   até `cmd_hold_s` (0,25 s), porque o detector roda a ~10 Hz e o `command_mux` zera
   a velocidade após 0,3 s sem comando.
7. **Métricas:** `t_c` é contado da primeira detecção válida até o erro entrar e
   permanecer na tolerância por `hold_time_s`; `e_r` é a média do erro nesse intervalo
   de permanência, não o último valor.
8. **Fuzzy fica para depois**, mas a troca precisa ser trivial: parâmetro `controller`
   e interface comum `compute(erro, dt) -> velocidade` definida em `controllers/base.py`.
   Implementar agora só o PID, com a fábrica já preparada para o segundo método.

## Ponto atual e próximos passos (08/10/2026)

**Feito:** percepção completa e testada no hardware (`perception.launch.py`), `yolo11n` em CPU
com o `equipment_coco_test.yaml`. Medidas de referência (`docs/testes.md` §2.2): `imgsz` 640 →
~10 quadros/s e ~115 ms de atraso; inferência pura 640 = 73 ms, 480 = 46 ms, 320 = 32 ms.

A malha de controle depende da distância focal em pixels, por isso a calibração entra ANTES
do `visual_servo_node`. As tarefas abaixo são sequenciais; uma por sessão.

### Tarefa A — `calibration_node` e `CameraInfo` (pré-requisito de tudo)

Especificação: `docs/architecture.md` §4.9 (nó) e §4.1 (publicação do `CameraInfo`).

Resumo do fluxo pretendido: o operador centraliza o alvo pela página web e zera os eixos
(`/ptu/set_zero`); depois chama `/calibration/run`, e o nó percorre sozinho uma grade de
ângulos em cruz, coleta amostras, ajusta o modelo, grava `config/camera_intrinsics.yaml`
e volta à posição inicial.

Pré-requisito: `command_mux` precisa aceitar `/ptu/cmd_pos_auto` (§4.6). Faça essa alteração
primeiro, isolada, e teste que o jog da web continua tendo prioridade.

Pontos de atenção:
- a imagem tem ~115 ms de atraso: descarte as mensagens de `/perception/target` cujo
  `header.stamp` seja anterior ao fim do movimento, senão a amostra mistura dois ângulos;
- o service bloqueia por dezenas de segundos: use `ReentrantCallbackGroup` com
  `MultiThreadedExecutor`, senão as assinaturas param durante a rotina;
- aborte a rotina se `/ptu/control_source` mudar para `web` (operador assumiu);
- devolva sempre os eixos à posição inicial, inclusive em caso de erro ou aborto.

### Tarefa B — medição no hardware

Rodar a calibração, conferir `f_x ≈ (W/2)/tan(FOV_h/2)` e registrar em `docs/testes.md`.
Resíduo RMS alto é sintoma de encoder ruim ou distorção, não de bug no ajuste.

### Tarefa C — `controllers/base.py` e `controllers/pid.py`

Lógica pura, sem ROS, com pytest, como foi feito em `scan_pattern.py`:
- `base.py`: interface `compute(erro, dt) -> velocidade`, `reset()` e a fábrica
  `criar_controlador(nome, params)`;
- `pid.py`: PID posicional com derivada sobre a medida filtrada (1ª ordem),
  anti-windup por congelamento do integrador na saturação e limite explícito do
  termo integral;
- testes: degrau, `dt` variável, saturação, não acumulação do integral saturado,
  entrada nula após `reset()`.

### Tarefa D — `visual_servo_node`

Especificação completa em `docs/architecture.md` §4.5. Pipeline de cinco estágios:
conversão → controlador → saturação → supervisão → publicação a 20 Hz.
Reaproveite do `scan_node` o padrão de action server (goal novo substitui o anterior,
cancelamento, zero ao terminar, `MultiThreadedExecutor` com `spin_once`).

O nó deve rejeitar o goal se ainda não recebeu `/camera/camera_info` válido
(`f_x > 0`), com mensagem explícita, em vez de assumir um valor.

Teste sem o manager:
```bash
ros2 service call /perception/set_target pantilt_interfaces/srv/SetTarget "{equipment: 'garrafa'}"
ros2 action send_goal -f /control/center pantilt_interfaces/action/Center \
  "{mode: 0, tolerance_px: 20.0, hold_time_s: 1.0, lost_timeout_s: 1.0}"
```

### Tarefa E — sinais, sintonia e ensaios

1. Com `kp = 0.2` e `ki = kd = 0`, verificar o sentido de giro e fixar
   `invert_pan` / `invert_tilt`. Fazer isso com a mão no botão de parada.
2. Subir `kp` até 0,8–1,0; observar sobressinal e oscilação.
3. Ativar `ki` só depois, no ensaio com alvo em movimento.
4. Registrar em `docs/testes.md`: sinais, CENTER, TRACK, alvo perdido, cancelamento,
   prioridade do operador e comportamento no limite de ângulo.
5. Gravar `ros2 bag record /perception/target /ptu/cmd_vel_auto /joint_states
   /camera/camera_info` para os gráficos do TCC.

**Depois disso**, na ordem da §12 do `architecture.md`: `inspection_manager`, teste da
inspeção pela página, `system.launch.py`, `controllers/fuzzy.py` e scripts de ensaio.

**Pendências sem prazo:**
- melhorias do `scan_node` em `docs/diagnostico_encoders.md` §4;
- comentário do `ListEquipment.srv`: ainda diz "cruzado com as classes do modelo". Ajustar na
  tarefa do `inspection_manager`, que lista só o yaml (architecture.md §4.3, v0.3).

## Armadilhas conhecidas

- **Webcam no WSL2:** no ambiente atual (kernel WSL 6.18) o driver UVC funciona: a câmera USB, anexada com `usbipd`, aparece como `/dev/video0` no container e entrega 640×480 MJPG a até 30 fps (menos com pouca luz, por causa da exposição automática). Use `source: "0"`. Em kernels sem UVC, o `camera_node` também aceita URL como `source` (stream MJPEG do Windows).
- **Imagens via DDS:** o SHM padrão do Fast DDS (512 KB) não comporta uma imagem 640×480 (921 KB). Sem ajuste, os quadros vão por UDP e se perdem em BEST_EFFORT. Todo processo ROS precisa de `FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/src/pantilt_ros/pantilt_bringup/config/fastdds.xml` e o container precisa de `/dev/shm` bem maior que 64 MB (ver `docs/ajustes_pantilt_dockerfile.md`). O `web_video_server` só recebe tópicos BEST_EFFORT com `qos_profile=sensor_data` na URL do stream, e não decodifica `%2F`: o tópico vai na URL com `/` literal.
- **Portas da web:** o docker-compose do `pantilt_dockerfile` publica só 8080 (página), 9090 (rosbridge) e 8081 (web_video_server). Esses são os padrões do `web.launch.py` e do `config.js`. Uma porta fora dessa lista funciona dentro do container (`curl`), mas não chega ao navegador do Windows.
- **Heartbeat serial:** o fail-safe do firmware é de 500 ms. O heartbeat do bridge deve ser de 5 Hz, nunca 2 Hz.
- **Protocolo serial:** definido em `pantilt_firmware/include/Serialprotocol.h`. Não altere tipos ou payloads sem alterar o firmware.
- **rosbridge + actions:** a web não usa actions diretamente; usa os services `/inspection/*` e o tópico `/inspection/status`.
- **Pesos `.pt`** não vão para o git (ver `.gitignore`). Ficam em `/ros2_ws/models/`; o `detector_node` não baixa nada (comando de download em `docs/testes.md` §2.2).
- **Detector só em CPU:** o container não tem CUDA e o WSL tem ~3,7 GB de RAM. O `detector_node` processa ~10 quadros/s a `imgsz` 640 e descarta de propósito os quadros que chegam durante a inferência. Toda malha que depende de `/perception/target` roda nessa taxa, com ~115 ms de atraso.
- **`f_x` depende da resolução:** vale para a resolução em que o centro da bbox é reportado. Se o `camera_node` mudar de resolução, a calibração precisa ser refeita. Trocar de câmera (OAK-D) invalida a calibração.
- **Atraso da imagem em rotinas que movem os eixos:** depois de mover, espere o eixo parar E descarte os alvos com `header.stamp` anterior à parada. Sem isso, calibração e medidas ficam enviesadas.
- **Telemetria = encoders:** a posição em `/joint_states` vem dos encoders AS5600, não da contagem de passos. Um encoder ruim produz um ângulo falso e plausível, e os limites do bridge e o `scan_node` confiam nele. Antes de qualquer malha fechada, confira `/joint_states` com jog curto (`docs/testes.md` §1.2).
- **Ctrl+C em nós com `SignalHandlerOptions.NO`:** o `KeyboardInterrupt` só chega quando a thread principal volta ao Python. Um `executor.spin()` sem timer fica bloqueado em C, e o nó não sai. Use laço com `spin_once(timeout_sec=0.1)`, como no `scan_node`.
- **CLI do ROS lenta:** no volume 9p, um `ros2 topic echo`/`hz` leva vários segundos para começar a receber. Timeouts curtos dão falsa impressão de tópico mudo; para medir, prefira um script `rclpy` ou espere mais.
- **Coleta de dataset e disco:** `/ros2_ws` é o `C:` do Windows, com pouco espaço livre. Nunca grave imagens cruas com `ros2 bag` por longos períodos (~20 MB/s); a coleta usa MP4.