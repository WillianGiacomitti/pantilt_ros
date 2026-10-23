# Roteiros de teste do pantilt_ros

Roteiros manuais para validar cada pacote com o hardware real: comandos para executar e o que conferir. Nenhum roteiro altera código.

## Sumário

- [0. Preparação comum](#0-preparação-comum)
- [1. pantilt_hardware](#1-pantilt_hardware)
  - [1.1 serial_bridge_node](#11-serial_bridge_node)
  - [1.2 Checagem da telemetria](#12-checagem-da-telemetria)
  - [1.3 Testes de segurança do bridge](#13-testes-de-segurança-do-bridge)
  - [1.4 command_mux](#14-command_mux)
- [2. pantilt_perception](#2-pantilt_perception)
  - [2.1 camera_node](#21-camera_node)
  - [2.2 detector_node](#22-detector_node)
  - [2.3 calibration_node](#23-calibration_node)
- [3. pantilt_web](#3-pantilt_web)
- [4. pantilt_control](#4-pantilt_control)
  - [4.1 scan_node](#41-scan_node)
  - [4.2 visual_servo_node](#42-visual_servo_node)
- [5. pantilt_dataset](#5-pantilt_dataset)
  - [5.1 capture_node](#51-capture_node)
  - [5.2 dataset.launch.py](#52-datasetlaunchpy)
  - [5.3 Painel de coleta](#53-painel-de-coleta)
  - [5.4 Sessão de coleta](#54-sessão-de-coleta)
- [6. pantilt_manager](#6-pantilt_manager)
  - [6.1 inspection_manager](#61-inspection_manager)
  - [6.2 system.launch.py](#62-systemlaunchpy)
- [7. pantilt_experiment](#7-pantilt_experiment)
  - [7.1 Sinais de diagnóstico (etapa 1)](#71-sinais-de-diagnóstico-etapa-1)
  - [7.2 experiment_recorder (etapa 2)](#72-experiment_recorder-etapa-2)
- [8. Solução de problemas](#8-solução-de-problemas)
- [9. Roteiros futuros](#9-roteiros-futuros)

| Pacote | O que valida | ESP32 | Câmera |
|---|---|---|---|
| `pantilt_hardware` | serial, telemetria, limites de ângulo, fail-safe, prioridade e watchdog do mux | sim (o mux pode ser testado sem ela) | não |
| `pantilt_perception` | `camera_node`: taxa, QoS, perda e retorno da câmera, `camera_info`; `detector_node`: detecções, filtro, alvo, imagem de debug, tempo de inferência; `calibration_node`: `f_x`/`f_y`, recusas e interrupções | só no `calibration_node` | sim (ou um vídeo gravado, exceto no `calibration_node`) |
| `pantilt_web` | vídeo na página, reconexão, jog com o hardware | só no jog | sim |
| `pantilt_control` | `scan_node`: varredura, cancelamento, prioridade do operador, abortos; `visual_servo_node`: sentido de giro, CENTER, TRACK, alvo perdido, limite, métricas | sim (o goal recusado sem calibração dispensa) | só no `visual_servo_node` |
| `pantilt_dataset` | `capture_node`: MP4 + `.json`, recusas, varredura durante a gravação, disco; `dataset.launch.py` e painel de coleta da página | só com varredura | sim |
| `pantilt_manager` | `inspection_manager`: recusas do início, varredura, confirmação, centralização, readquisição, operador, stop e encerramento | sim | sim |
| `pantilt_experiment` | sinais de diagnóstico do servo e do detector; `experiment_recorder`: pastas por inspeção, CSVs, `resumo.json`, rótulo e repetição, recusas | sim (só os testes automáticos dispensam) | sim |

Ordem recomendada na primeira vez: 1.1 → 1.2 → 1.3 → 1.4 → 4 → 5. A seção 1.2 é pré-requisito de qualquer teste em malha fechada (seções 4 e 5); a 4.2 exige também a 2.2 e a calibração da 2.3. As seções 5.1 a 5.3 foram aprovadas no hardware (29/09 e 02/10/2026). Para validar o `detector_node`, siga 2.2 e depois a seção 3 sem `?video_topic=`. A seção 6 exige a 2.2, a 2.3 e os sinais da 4.2. A seção 7 exige a 6.1.

---

## 0. Preparação comum

### Antes de subir o container (Windows)

```powershell
usbipd list                              # anote o BUSID da ESP32 e o da câmera
usbipd attach --wsl --busid <BUSID>      # um attach por dispositivo
```

### Dentro do container (todo terminal novo)

```bash
docker exec -it ptu_web_bridge bash
ws                                        # entra em /ros2_ws e carrega o ambiente
```

### Conferência do ambiente (uma vez por sessão)

```bash
# setuptools: a correção se perde se o container foi recriado
pip show setuptools | grep Version        # precisa ser 58.2.0
pip install setuptools==58.2.0            # só se não for

# transporte de imagens (ver docs/ajustes_pantilt_dockerfile.md)
echo $FASTRTPS_DEFAULT_PROFILES_FILE      # .../pantilt_bringup/config/fastdds.xml
df -h /dev/shm                            # Size 1.0G

# dispositivos
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-id/ 2>/dev/null   # ESP32
ls /sys/class/video4linux/                # video0 e video1: câmera anexada de fato

# nenhum nó antigo rodando (ex.: iniciado pelo entrypoint)
ps aux | grep -v grep | grep -E 'serial_bridge|command_mux|camera_node|detector_node|scan_node|visual_servo|capture_node|inspection_manager|web_video_server'

# build
colcon build --symlink-install && ws
ros2 daemon stop                          # o daemon reinicia com o perfil do Fast DDS
```

Se o build falhar com `canonicalize_version() ... strip_trailing_zero`, o setuptools voltou ao 84: repita o `pip install` acima.

O `ls /dev/video*` sozinho não basta: o nó `/dev/video0` pode continuar existindo depois que a câmera foi desanexada. O `/sys/class/video4linux/` mostra o estado real.

### Parâmetros

Todos os nós leem o `pantilt_bringup/config/params.yaml`. Com `--symlink-install`, uma mudança nele vale sem rebuild: basta reiniciar o launch. Para testar sem mexer no arquivo do repositório, use uma cópia:

```bash
cp /ros2_ws/src/pantilt_ros/pantilt_bringup/config/params.yaml /tmp/params_teste.yaml
# edite /tmp/params_teste.yaml, por exemplo: device: "/dev/ttyACM0"
ros2 launch pantilt_bringup hardware.launch.py params_file:=/tmp/params_teste.yaml
```

### Referências rápidas

Conversão (os tópicos usam rad): 5° = 0.0873 · 10° = 0.1745 · 20° = 0.3491 · 29° = 0.5061 · 1 rad = 57.3°.

**Comando de parada** (deixe já digitado num terminal durante qualquer teste com movimento):

```bash
ros2 topic pub --once /ptu/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.0, y: 0.0}}"
```

Com a página aberta, o botão STOP faz o mesmo.

**CLI lenta:** no volume 9p, um `ros2 topic echo`/`hz` leva vários segundos para começar a receber. Não conclua que um tópico está mudo por um timeout curto.

**Ctrl+C num launch:** pode aparecer um `KeyboardInterrupt` no traceback do `serial_bridge_node`. O `ros2 launch` repassa um segundo SIGINT enquanto o nó já está encerrando. A velocidade zero é enviada antes desse ponto (`destroy_node`), então o eixo para mesmo assim.

---

## 1. pantilt_hardware

**O que valida:** a comunicação com a ESP32 (`serial_bridge_node`), a telemetria (contagem de passos), os limites de ângulo e o fail-safe, e a arbitração de comandos (`command_mux`).

Os dois nós sobem juntos pelo `hardware.launch.py`.

### 1.1 serial_bridge_node

**Terminal 1: launch**

```bash
ros2 launch pantilt_bringup hardware.launch.py
```

Esperado no log:

- `[serial_bridge_node]: Limites efetivos: pan [-29.0°, 29.0°], tilt [-89.0°, 89.0°]`;
- `[serial_bridge_node]: Conectado a /dev/ttyUSB0 @ 921600 bps` e, se a ESP32 reiniciar ao abrir a porta, `[ESP32] Boot detectado ...`;
- `[command_mux]: Prioridade do operador: 1.00 s | watchdog de comando: 0.30 s`;
- `[command_mux]: Fonte de controle inicial: none`.

**Terminal 2: observação**

```bash
ros2 node list                              # /serial_bridge_node e /command_mux
ros2 node info /serial_bridge_node          # assina /ptu/cmd_vel e /ptu/cmd_pos; publica /joint_states e /ptu/errors; serviço /ptu/set_zero
ros2 param dump /serial_bridge_node         # confira heartbeat_hz: 5.0 e os limites

ros2 topic hz /joint_states                 # ~20 Hz (taxa da telemetria do firmware)
ros2 topic echo /joint_states --field position   # [pan, tilt] em rad
ros2 topic echo /ptu/errors                 # diagnósticos da ESP32 (transient_local: mostra também os antigos)
```

Deixe o `/ptu/errors` aberto num terminal durante todo o teste. Para ver os comandos que chegam ao bridge: `ros2 topic echo /ptu/cmd_vel` e `ros2 topic echo /ptu/cmd_pos`.

**Terminal 3: acionamento**

> ⚠️ **Aqui os comandos vão direto para o bridge (`/ptu/cmd_vel` e `/ptu/cmd_pos`), contornando o `command_mux`: não há watchdog de comando.** O firmware mantém a última velocidade recebida, e o heartbeat do bridge impede o fail-safe. Depois de qualquer comando de velocidade, **sempre envie o comando de parada**. Prefira `--once` a `-r`: interromper um `-r 10` com Ctrl+C **não** para o eixo.

Velocidade (`angular.z` = pan, `angular.y` = tilt, em rad/s; comece devagar):

```bash
ros2 topic pub --once /ptu/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.1}}"    # pan +5.7°/s
ros2 topic pub --once /ptu/cmd_vel geometry_msgs/msg/Twist "{angular: {z: -0.1}}"   # pan -5.7°/s
ros2 topic pub --once /ptu/cmd_vel geometry_msgs/msg/Twist "{angular: {y: 0.1}}"    # tilt +5.7°/s
ros2 topic pub --once /ptu/cmd_vel geometry_msgs/msg/Twist "{angular: {y: -0.1}}"   # tilt -5.7°/s
```

No primeiro comando, anote para que lado o eixo gira com o sinal positivo e se o `/joint_states` cresce.

Posição (absoluta, em rad):

```bash
ros2 topic pub --once /ptu/cmd_pos sensor_msgs/msg/JointState "{name: [pan_joint, tilt_joint], position: [0.1745, 0.0]}"  # pan 10°, tilt 0°
ros2 topic pub --once /ptu/cmd_pos sensor_msgs/msg/JointState "{name: [tilt_joint], position: [0.1745]}"               # só tilt 10°; pan fica onde está
ros2 topic pub --once /ptu/cmd_pos sensor_msgs/msg/JointState "{name: [pan_joint, tilt_joint], position: [0.0, 0.0]}"     # volta ao zero
```

Zerar os eixos:

```bash
ros2 service call /ptu/set_zero std_srvs/srv/Trigger     # esperado: success=True, message='Eixos zerados'
```

### 1.2 Checagem da telemetria

Confirma que a posição em `/joint_states` (contagem de passos do firmware, relativa ao zero manual) acompanha o movimento real. **Não use `/control/scan` nem nada em malha fechada antes de estes testes passarem nos dois eixos.** Sem encoder, um passo perdido ou uma escala errada (micropasso, redução) produz um ângulo falso e plausível, e os limites do bridge e o `scan_node` confiam nele. Confira o ângulo real com uma referência externa (ex.: transferidor ou inclinômetro do celular).

Com o launch da seção 1.1 rodando:

```bash
ros2 topic echo /joint_states --field position     # [pan, tilt] em rad
ros2 topic echo /joint_states --field velocity
```

Use velocidades baixas (jog da página ou comandos da seção 1.1) e o comando de parada à mão.

| # | Teste | Esperado |
|---|---|---|
| 1 | Parado por 30 s | posição estável: variação menor que ~0,2° |
| 2 | Jog curto no pan (+ e depois −) | a posição do pan acompanha o movimento e volta |
| 3 | Jog curto no tilt | idem no tilt; o pan não muda |
| 4 | `cmd_pos` para 10° e depois para 0° | a telemetria chega a ~10° e volta a ~0° |
| 5 | Velocidade com o eixo parado | ~0 rad/s em repouso |

### 1.3 Testes de segurança do bridge

Os limites de software são **relativos ao zero**. Antes destes testes, leve o mecanismo ao centro mecânico real e zere (`/ptu/set_zero`); caso contrário, os ±29° ficam deslocados.

Faça nesta ordem:

| # | Teste | Comando | Esperado |
|---|---|---|---|
| 1 | Heartbeat | nó parado por ~1 min | nenhum `Fail-safe acionado` em `/ptu/errors` |
| 2 | Recorte de posição | `cmd_pos` com `position: [1.0, 0.0]` | pan vai a ~29° (0.506 rad); log `Comando de posição recortado ao limite` |
| 3 | Limite por velocidade | volte ao zero e envie `cmd_vel` `{angular: {z: 0.1}}` **sem** parar | pan para sozinho antes de 29°; log `Limite de ângulo atingido` |
| 4 | Bloqueio só no sentido do limite | com pan em ~29°: `z: 0.1` e depois `z: -0.1` | `+0.1` ignorado (log `Comando de velocidade limitado`); `-0.1` afasta do limite (depois pare) |
| 5 | Parada no Ctrl+C | eixo em movimento lento → Ctrl+C no terminal 1 (encerra o launch) | eixo para imediatamente (o bridge envia velocidade zero ao sair) |
| 6 | Fail-safe do firmware | suba o launch de novo; eixo em movimento → `pkill -9 -f 'lib/pantilt_hardware/serial_bridge_node'` | eixo para em ~0,5 s (quem para é o fail-safe, sem o zero do bridge); o launch registra `process has died` e o `command_mux` continua rodando |
| 7 | Reconexão | desconecte o USB com o nó rodando e reconecte (e refaça o `usbipd attach`) | `/ptu/errors`: `Conexão serial ... perdida - reconectando...` e depois `Conectado a ...`; o caminho da porta precisa ser o mesmo |
| 8 | Parâmetro inválido | com o launch parado: `ros2 run pantilt_hardware serial_bridge_node --ros-args -p heartbeat_hz:=2.0` | `[FATAL] heartbeat_hz=2.0 inválido ...` e o nó sai |

O teste 8 usa `ros2 run` de propósito: o `-p` sobrescreve um único parâmetro, sem editar o `params.yaml`.

Para registrar o ensaio (opcional): `ros2 bag record /joint_states /ptu/cmd_vel /ptu/cmd_pos /ptu/errors`.

### 1.4 command_mux

O `command_mux` é o único caminho de comandos até o bridge (architecture.md §4.6). Ele assina `/ptu/cmd_vel_auto`, `/ptu/cmd_vel_web`, `/ptu/cmd_pos_web` e `/ptu/cmd_pos_auto`, repassa para `/ptu/cmd_vel` e `/ptu/cmd_pos` e publica a fonte ativa (`web`, `auto` ou `none`) em `/ptu/control_source`.

- **Prioridade:** um comando da web é repassado na hora. Os comandos automáticos (velocidade e posição) são descartados enquanto a web tiver publicado nos últimos `operator_hold_s` (1,0 s).
- **Watchdog:** depois de uma velocidade **não nula**, se a fonte ativa ficar em silêncio por mais de `cmd_timeout_s` (0,3 s), o mux envia velocidade zero uma única vez. Comandos de posição não armam o watchdog, porque um zero interromperia o movimento de posição.

Estes testes não precisam da ESP32. Sem ela, o `serial_bridge_node` fica registrando `Falha ao abrir /dev/ttyUSB0` a cada tentativa de reconexão; isso é esperado e não afeta o mux. Com o hardware, observe o eixo.

**Terminal 1:** `ros2 launch pantilt_bringup hardware.launch.py` (o mesmo da seção 1.1).

**Terminal 2: observação**

```bash
ros2 node info /command_mux                 # assina os 4 tópicos de entrada; publica /ptu/cmd_vel, /ptu/cmd_pos, /ptu/control_source
ros2 topic echo /ptu/control_source         # transient_local: mostra a fonte atual ao conectar
ros2 topic echo /ptu/cmd_vel --field angular
ros2 topic echo /ptu/cmd_pos --field position   # testes 7 e 8
```

**Terminal 3: acionamento.** Agora use os tópicos de entrada do mux, **nunca** `/ptu/cmd_vel` direto.

| # | Teste | Comando | Esperado |
|---|---|---|---|
| 1 | Repasse automático | `ros2 topic pub -r 10 /ptu/cmd_vel_auto geometry_msgs/msg/Twist "{angular: {z: 0.1}}"` | `/ptu/cmd_vel` recebe `z: 0.1` a 10 Hz; fonte `auto` |
| 2 | Watchdog auto | Ctrl+C no `pub` do teste 1 | em ~0,3 s um **único** `z: 0.0` em `/ptu/cmd_vel`; log `Watchdog: fonte "auto" em silêncio ...`; fonte `none`. Com hardware, o eixo para sozinho |
| 3 | Prioridade do operador | repita o `pub` do teste 1 e, em outro terminal: `ros2 topic pub --once /ptu/cmd_vel_web geometry_msgs/msg/Twist "{angular: {y: 0.1}}"` | fonte `web` na hora; `y: 0.1` repassado; o auto é descartado por 1 s (log `Comando automático descartado`); após 0,3 s sai o zero do watchdog (`fonte "web"`); passado 1 s, o auto volta (fonte `auto`) |
| 4 | Posição sem watchdog | `ros2 topic pub --once /ptu/cmd_pos_web sensor_msgs/msg/JointState "{name: [pan_joint, tilt_joint], position: [0.1745, 0.0]}"` | repassado em `/ptu/cmd_pos`; fonte `web` por 1 s e depois `none`; **nenhum** zero em `/ptu/cmd_vel` (com hardware, o eixo chega aos 10°) |
| 5 | Parada no Ctrl+C | com o `pub` do teste 1 rodando, Ctrl+C no terminal 1 (encerra o launch) | log `Encerrando: enviando velocidade zero` do mux; `/ptu/cmd_vel` recebe `z: 0.0`; o bridge também envia zero ao sair |
| 6 | Parâmetro inválido | com o launch parado: `ros2 run pantilt_hardware command_mux --ros-args -p cmd_timeout_s:=0.0` | `[FATAL] Parâmetro inválido: cmd_timeout_s deve ser positivo ...` e o nó sai |
| 7 | Posição automática | `ros2 topic pub --once /ptu/cmd_pos_auto sensor_msgs/msg/JointState "{name: [pan_joint, tilt_joint], position: [0.1745, 0.0]}"` | repassado em `/ptu/cmd_pos`; fonte `auto` por 0,3 s e depois `none`; **nenhum** zero em `/ptu/cmd_vel` (com hardware, o pan chega aos 10°) |
| 8 | Prioridade sobre a posição automática | jog contínuo pela página (ou `ros2 topic pub -r 10 /ptu/cmd_vel_web geometry_msgs/msg/Twist "{angular: {z: 0.05}}"`) e, durante o jog, o comando do teste 7 | log `Comando automático de posição descartado: operador no controle`; nada novo em `/ptu/cmd_pos`; o jog continua; fonte `web` |

Uma posição automática logo depois de uma velocidade não nula desarma o watchdog: não sai zero que interrompa o movimento. A janela de 0,3 s é curta demais para a CLI; o caso é coberto pelo pytest (`test_posicao_auto_desarma_velocidade_anterior`).

Para registrar: `ros2 bag record /ptu/cmd_vel_auto /ptu/cmd_vel_web /ptu/cmd_pos_web /ptu/cmd_pos_auto /ptu/cmd_vel /ptu/cmd_pos /ptu/control_source /joint_states`.

### Preste atenção

- **Nenhum `ERRO 4: Fail-safe acionado`** pode aparecer em `/ptu/errors` com o nó parado. Se aparecer, o heartbeat não está a 5 Hz.
- Os erros 12 a 17 do firmware (driver, temperatura, limite do firmware) aparecem em `/ptu/errors` com a descrição; os de driver se repetem a cada 2 s enquanto a condição durar. Os códigos de encoder, passos perdidos e home (1, 2, 8 a 11 e 18) são legado e não devem aparecer (architecture.md §8). `Código de erro desconhecido (N)` indica que o firmware tem um código novo que o `serial_protocol.py` ainda não conhece.
- Perto de ±29° podem surgir os erros 16/17 (limite de software **do firmware**). Anote qual limite age primeiro, o do bridge ou o do firmware.
- Se no teste 3 da seção 1.3 o pan passar de 29° de forma relevante, aumente `limit_margin_deg` no `params.yaml`.
- Limitação conhecida: se o mux for morto com `kill -9` com o eixo em movimento, nada envia o zero, e o heartbeat do bridge impede o fail-safe do firmware. Só os limites de ângulo param o eixo nesse caso.

---

## 2. pantilt_perception

O `perception.launch.py` sobe o `camera_node` e o `detector_node` juntos (seção 2.2). Para testar só a câmera, use o `ros2 run` da seção 2.1.

### 2.1 camera_node

**O que valida:** o `camera_node` publicando `/camera/image_raw` com a taxa e o QoS certos, o comportamento quando a câmera some e volta, e os intrínsecos em `/camera/camera_info` (architecture.md §4.1).

**Terminal 1: câmera.** O nó sobe sozinho com `ros2 run`:

```bash
ros2 run pantilt_perception camera_node --ros-args \
  --params-file /ros2_ws/src/pantilt_ros/pantilt_bringup/config/params.yaml
```

Esperado no log:

```
Fonte: device "0" | pedido 640x480 @ 30.0 fps | frame_id "camera_optical_frame"
Fonte aberta: 640x480 @ 30.0 fps (MJPG)
```

Antes da primeira calibração (seção 2.3), também aparece este aviso, uma única vez. É esperado:

```
Sem arquivo de calibração (/ros2_ws/.../config/camera_intrinsics.yaml): CameraInfo com K zerada; rode o calibration_node
```

Se aparecer `A fonte entrega WxH em vez de 640x480`, a câmera não aceitou a resolução pedida. O nó publica no tamanho nativo.

**Terminal 2: verificação**

```bash
ros2 param dump /camera_node                # confere que o params.yaml foi lido
ros2 topic hz /camera/image_raw             # ~19-30 Hz conforme a luz; ~7 Hz indica problema de transporte (seção 8)
ros2 topic info -v /camera/image_raw        # publisher camera_node: Reliability BEST_EFFORT
ros2 topic echo /camera/image_raw --field header   # stamp avançando e frame_id camera_optical_frame
ros2 topic hz /camera/camera_info           # mesma taxa da imagem
ros2 topic echo /camera/camera_info --field k      # 9 zeros sem calibração
ros2 topic echo /camera/camera_info --field header # mesmo stamp e frame_id da imagem
```

Anote a taxa desta etapa: ela é a referência para a seção 3.

| # | Teste | Como | Esperado |
|---|---|---|---|
| 1 | Parar a câmera | Ctrl+C no terminal 1 | com a página aberta, o vídeo congela no último quadro (anote se a página muda para `Aguardando imagens`) |
| 2 | Voltar a câmera | suba o `camera_node` de novo | anote se o vídeo da página volta sozinho ou só ao recarregar |
| 3 | Câmera desconectada | `usbipd detach --busid <BUSID>` no Windows | log `Fonte "0" parou de entregar quadros; reabrindo` e depois `Não foi possível abrir a fonte "0"` a cada 5 s |
| 4 | Câmera reconectada | `usbipd attach --wsl --busid <BUSID>` | anote se o nó volta a abrir a fonte sozinho (`Fonte aberta: ...`) ou se precisa ser reiniciado |

**Intrínsecos (`/camera/camera_info`).** Para os testes 6 a 8, crie um arquivo de teste (valores fictícios; o real vem da seção 2.3):

```bash
cat > /tmp/intr_teste.yaml <<'FIM'
image_width: 640
image_height: 480
camera_name: pantilt_cam
camera_matrix:
  rows: 3
  cols: 3
  data: [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]
FIM
```

E suba o nó apontando para ele (Ctrl+C no terminal 1 antes):

```bash
ros2 run pantilt_perception camera_node --ros-args \
  --params-file /ros2_ws/src/pantilt_ros/pantilt_bringup/config/params.yaml \
  -p camera_info_file:=/tmp/intr_teste.yaml
```

| # | Teste | Como | Esperado |
|---|---|---|---|
| 5 | Sem calibração | comando padrão do terminal 1 (sem `camera_intrinsics.yaml`) | aviso `Sem arquivo de calibração ...` **uma vez**; `--field k` com 9 zeros; `width: 640`, `height: 480`; a imagem segue normal |
| 6 | Com calibração | comando acima | log `Calibração: fx=600.0, fy=600.0, cx=320.0, cy=240.0 (640x480) de /tmp/intr_teste.yaml`; `k: [600, 0, 320, 0, 600, 240, 0, 0, 1]`; `distortion_model: plumb_bob` |
| 7 | Mesmo header da imagem | com o teste 6 rodando, compare `ros2 topic echo /camera/camera_info --field header` com o de `/camera/image_raw` | mesmo `frame_id` e os mesmos valores de `stamp` (o `camera_info` sai logo depois da imagem) |
| 8 | Resolução diferente da calibração | comando acima com `-p width:=320 -p height:=240` | ao abrir a fonte, `Calibração feita para 640x480, mas a fonte entrega 320x240: valores não reescalados; refaça a calibração`; `K` continua a do arquivo |
| 9 | Arquivo inválido | `echo "image_width: [640" > /tmp/intr_ruim.yaml` e suba com `-p camera_info_file:=/tmp/intr_ruim.yaml` | `[ERROR] Arquivo de calibração inválido: YAML inválido ...; CameraInfo com K zerada`; o nó **continua** publicando a imagem |

Para registrar (opcional): `ros2 bag record /camera/image_raw`. Ocupa ~20 MB/s a 640×480 e 20 Hz; o `/ros2_ws` fica no `C:` do Windows, com pouco espaço. Grave poucos segundos.

### Preste atenção

- O nó precisa ser iniciado com `--ros-args --params-file`. Um `ros2 param dump` com valores diferentes do yaml indica que o arquivo não foi lido.
- Taxa em ~7 Hz é problema de transporte (perfil do Fast DDS ou `/dev/shm`), não da câmera.
- Com pouca luz, a exposição automática derruba a taxa. Compare as medidas sempre com a mesma iluminação.
- O arquivo de calibração é lido só no início. Depois de calibrar (seção 2.3), **reinicie o `camera_node`** para publicar a `K` nova.
- A calibração vale só para a resolução em que foi feita. O aviso do teste 8 indica que é preciso calibrar de novo.

### 2.2 detector_node

**O que valida:** o `detector_node` (architecture.md §4.2):
- a YOLO sobre `/camera/image_raw`;
- as detecções em `/perception/detections`;
- o filtro por `/perception/set_target`;
- o alvo em `/perception/target`;
- a imagem com as caixas em `/perception/debug_image`;
- o tempo de inferência em CPU.

Não precisa da ESP32. A fonte pode ser a câmera ou um vídeo já gravado em `/ros2_ws/datasets` (ensaio repetível).

**Preparação (uma vez):** os pesos ficam em `/ros2_ws/models/`, fora do git, e o nó **não baixa** nada. Para o modelo de teste COCO:

```bash
mkdir -p /ros2_ws/models && cd /ros2_ws/models && python3 -c "from ultralytics import YOLO; YOLO('yolo11n.pt')"
ls -la /ros2_ws/models/                   # yolo11n.pt, ~5,4 MB
```

O `params.yaml` aponta para esse arquivo e para o `equipment_coco_test.yaml`:
- `garrafa` = `bottle`;
- `copo` = `cup`;
- `celular` = `cell phone`.

Depois do treino, troque `model_path` e `equipment_file` (para `.../config/equipment.yaml`).

**Terminal 1: percepção**

```bash
ros2 launch pantilt_bringup perception.launch.py
```

Esperado no log do detector (o carregamento leva alguns segundos):

```
Modelo /ros2_ws/models/yolo11n.pt carregado e aquecido em 4.8 s (80 classes)
Equipamentos disponíveis: garrafa (bottle), copo (cup), celular (cell phone)
Pronto: imgsz 640 | conf 0.50 | device "cpu" | debug_image sim | sem filtro
```

A cada 10 s:

```
10.6 quadros/s processados | inferência média 93 ms | atraso médio desde a captura 114 ms
```

**Terminal 2: observação**

```bash
ros2 topic echo /perception/detections --field detections   # class_id (nome) e score; bbox em px
ros2 topic echo /perception/target                           # só com filtro
ros2 topic hz /perception/detections                         # ~ taxa do log
```

**Terminal 3: filtro**

```bash
ros2 service call /perception/set_target pantilt_interfaces/srv/SetTarget "{equipment: 'garrafa'}"
ros2 service call /perception/set_target pantilt_interfaces/srv/SetTarget "{equipment: ''}"   # remove
```

**Terminal 4 (opcional): página.** `ros2 launch pantilt_web web.launch.py` e `http://localhost:8080/`, sem `?video_topic=`: o vídeo mostra o `/perception/debug_image`.

| # | Teste | Como | Esperado |
|---|---|---|---|
| 1 | Sem filtro | ponha pessoas e objetos diante da câmera | `/perception/detections` com todas as classes (ex.: `person`, `chair`, `bottle`); **nada** em `/perception/target` |
| 2 | Imagem de debug | página aberta | caixas azuis com `classe confiança`, cruz no centro, faixa `Sem filtro \| N det. \| X ms`; chip de detecções sob o vídeo |
| 3 | Filtro garrafa | `set_target` com `garrafa` | `success=True`, `Filtro: Garrafa (teste COCO) (bottle)`; `detections` só com `bottle`; `/perception/target` a cada quadro, com `equipment: garrafa` |
| 4 | Alvo some e volta | esconda e mostre a garrafa | `detected: false` sem garrafa (com `image_width`/`image_height` preenchidos) e `true` com ela |
| 5 | Sinal do erro | garrafa à direita do centro, depois abaixo | `error_x > 0` à direita; `error_y > 0` abaixo; perto da cruz, os dois perto de 0 |
| 6 | Maior score | duas garrafas, uma perto (grande) e outra longe (pequena) | o alvo (caixa laranja, linha até o centro) é a de maior confiança × área, normalmente a grande |
| 7 | Chave desconhecida | `set_target` com `xyz` | `success=False`, `equipamento "xyz" desconhecido. Disponíveis: garrafa, copo, celular` |
| 8 | Remover o filtro | `set_target` com `''` | `Filtro removido: publicando todas as classes`; `/perception/target` para; `detections` volta a todas as classes |
| 9 | Câmera parada | `pkill -INT -f lib/pantilt_perception/camera_node` | o detector para de publicar sem erro; em até 10 s, `Nenhum quadro de /camera/image_raw em 10 s (a câmera está rodando?)` |
| 10 | Modelo ausente | launch parado: `ros2 run pantilt_perception detector_node --ros-args --params-file <params.yaml> -p model_path:=/nao/existe.pt` | `[FATAL] Parâmetro inválido: model_path "/nao/existe.pt" não encontrado. O nó não baixa pesos; ...` com o comando de download; o nó sai |
| 11 | Parâmetro inválido | idem, com `-p imgsz:=600` | `[FATAL] Parâmetro inválido: imgsz=600 inválido: deve ser positivo e múltiplo de 32` |
| 12 | Tempo por `imgsz` | repita o launch com uma cópia do `params.yaml` (seção 0) com `imgsz` 640, 480 e 320 | anote a taxa, a inferência e o atraso do log em cada caso, com a mesma cena |
| 13 | Ensaio com vídeo gravado | cópia do `params.yaml` com `source: "/ros2_ws/datasets/<arquivo>.mp4"` no `camera_node` | o vídeo roda em laço; as detecções se repetem a cada volta (ensaio repetível, sem câmera) |

Referência medida no desenvolvimento (02/10/2026), na CPU do container, sem ROS no meio: `imgsz` 640 = 73 ms, 480 = 46 ms e 320 = 32 ms por quadro. Com o nó completo, 640 deu ~10 quadros/s e ~115 ms de atraso desde a captura.

Para registrar (opcional): `ros2 bag record /perception/detections /perception/target`. São mensagens pequenas; não grave a `debug_image`.

### Preste atenção

- **Só CPU:** o container não tem CUDA (`device: "cpu"`). O detector processa menos quadros que a câmera entrega. Os quadros que chegam durante uma inferência são **descartados de propósito**, para o alvo não ficar atrasado. A taxa do `/perception/detections` é a taxa do detector, não a da câmera.
- **Memória:** a RAM do WSL é de ~3,7 GB. Com tudo rodando, confira `free -h`; se o sistema ficar lento, feche o navegador extra ou reduza o `imgsz`.
- **Atraso:** o número do log (agora − instante da captura) é o que a malha IBVS vai sentir. Anote-o junto com o `imgsz` no teste 12: ele entra na escolha dos ganhos do `visual_servo_node`.
- O `/perception/set_target` responde depois da inferência em andamento (até ~0,1 s).
- O ultralytics cria `~/.config/Ultralytics/` na primeira execução. É esperado.
- Classes do COCO estão em inglês (`bottle`, `cell phone`). O `class_name` do yaml precisa ser idêntico ao do modelo; um nome errado aparece no log como `Equipamento "..." ignorado: a classe "..." não existe no modelo`.

### 2.3 calibration_node

**O que valida:** o `calibration_node` (architecture.md §4.10):
- a estimativa de `f_x` e `f_y` girando a câmera em ângulos conhecidos;
- a gravação do `camera_intrinsics.yaml`;
- o retorno ao *home*;
- as recusas e as interrupções.

Precisa da ESP32 e da câmera. **Faça antes a seção 1.2:** o ajuste confia no ângulo da telemetria (contagem de passos).

**Preparação** (um terminal para cada comando, todos com a seção 0 aplicada):

```bash
ros2 launch pantilt_bringup hardware.launch.py
ros2 launch pantilt_bringup perception.launch.py
ros2 launch pantilt_web web.launch.py
ros2 run pantilt_perception calibration_node --ros-args \
  --params-file /ros2_ws/src/pantilt_ros/pantilt_bringup/config/params.yaml
```

Depois, no terminal de comandos:

1. Ponha um objeto da lista de equipamentos (ex.: uma garrafa) parado, a 1–3 m, com fundo limpo.
2. `ros2 service call /perception/set_target pantilt_interfaces/srv/SetTarget "{equipment: 'garrafa'}"`
3. Pela página, centralize o alvo com o jog. Bastam ±20% da meia-largura e da meia-altura (±64 e ±48 px a 640×480).
4. Zere os eixos pela página (botão de zero) ou com `ros2 service call /ptu/set_zero std_srvs/srv/Trigger`.
5. Solte o jog e espere 1 s: a fonte precisa sair de `web`.

**Execução:**

```bash
ros2 service call /calibration/run std_srvs/srv/Trigger   # responde só no fim (~30-60 s)
```

O nó move o pan ±3° (sondagem) e percorre 7 pontos no pan e depois 7 no tilt, sempre um eixo por vez. Por fim volta ao *home*. No log aparece uma linha por ponto e os resíduos do ajuste. A resposta traz:

```
Calibração gravada em /ros2_ws/src/pantilt_ros/pantilt_bringup/config/camera_intrinsics.yaml
pan:  fx = ... px | RMS ... px | 7 pontos | FOV_h ...° | pan+ desloca o alvo para a ... na imagem
tilt: fy = ... px | RMS ... px | 7 pontos | FOV_v ...° | tilt+ desloca o alvo para ... na imagem
cx = 320.0, cy = 240.0 (centro). Reinicie o camera_node para publicar a K nova.
```

| # | Teste | Como | Esperado |
|---|---|---|---|
| 1 | Calibração normal | preparação + `run` | `success: True`; os eixos param em cada ponto e voltam ao *home*; arquivo gravado; RMS de poucos px |
| 2 | `K` nova publicada | reinicie o `perception.launch.py` e rode `ros2 topic echo /camera/camera_info --field k` | log `Calibração: fx=...`; `k` com os valores da resposta e `cx`, `cy` no centro |
| 3 | Sem filtro | `set_target` com `''` e `run` | `Recusada: nenhum /perception/target recebido ...` ou `último /perception/target tem N s ...` |
| 4 | Alvo fora do centro | desloque o alvo para a borda e `run` | `Recusada: alvo fora do centro (erro ...)`; nada se move |
| 5 | Eixos fora de zero | jog de ~5° no pan e `run` | `Recusada: eixos fora de zero (pan 5.0°, ...)`; nada se move |
| 6 | Jog ativo | segure o jog e, em outro terminal, `run` | `Recusada: operador no controle ...` (a CLI pode atrasar: o jog precisa estar ativo quando a chamada chega) |
| 7 | Abort no meio | `run` e, durante a grade, `ros2 service call /calibration/abort std_srvs/srv/Trigger` | abort: `Abortando: os eixos voltam ao home`; run: `Calibração interrompida: abortada ... (eixos de volta ao home)`; eixos no *home* |
| 8 | Operador assume no meio | `run` e, durante a grade, um toque no jog da página | `Calibração interrompida: operador assumiu o controle ... (eixos não foram movidos de volta)`; os eixos **não** voltam sozinhos ao *home* |
| 9 | Alvo retirado | `run` e, durante a grade do tilt, tire o objeto de cena | avisos `... ponto descartado`; com menos de 4 pontos válidos, `só N pontos válidos no tilt ...` e volta ao *home* |
| 10 | Ctrl+C no meio | `run` e Ctrl+C no terminal do `calibration_node` | o nó sai; os eixos param no último ponto e **não** voltam ao *home* |
| 11 | Abort sem rotina | `abort` com o nó ocioso | `success: False`, `Nenhuma calibração em andamento` |

Para registrar: `ros2 bag record /perception/target /joint_states /ptu/cmd_pos_auto /ptu/control_source`.

### Preste atenção

- **Interpretação** (architecture.md §4.10):
  - RMS abaixo de ~2 px é bom;
  - resíduos grandes e sistemáticos nas pontas indicam distorção: reduza `max_angle_deg`;
  - resíduos grandes e dispersos indicam passos perdidos;
  - `f_x` longe de `(W/2)/tan(FOV_h/2)` em mais de ~15% é motivo para desconfiar.
- **Sentido dos eixos:** a resposta diz para que lado o alvo anda com cada eixo positivo. Anote: isso adianta o `invert_pan`/`invert_tilt` da tarefa E.
- **Junta que não para:** `o pan parou em X°, a Y° do alvo ...` mostra que o firmware para fora da tolerância. Aumente `settle_tol_deg`; o ajuste usa o ângulo medido, não o comandado. Já `não parou ... (velocidade V°/s)` com o eixo visivelmente parado indica velocidade da telemetria ruidosa em repouso (seção 1.2, teste 5).
- **Resolução:** a calibração vale para a resolução do quadro (640×480). Mudou a resolução ou a câmera, calibre de novo.
- O `run` bloqueia o terminal até o fim. Use outro terminal para o `abort`.
- O arquivo é gravado em `pantilt_bringup/config` (dentro do repositório). Decida se ele entra no commit.

---

## 3. pantilt_web

**O que valida:** o caminho `camera_node` → `/camera/image_raw` → `web_video_server` → página, a reconexão da página ao rosbridge e o jog com o hardware (a imagem se move junto com o pan-tilt).

A página mostra o `/perception/debug_image` (caixas do `detector_node`, seção 2.2). Para testar só a câmera, sem o detector, abra com `?video_topic=/camera/image_raw`. A aba Coleta já usa `/camera/image_raw` sozinha (seção 5.3); o `?video_topic=`, quando presente, vale para as duas abas.

Endereços (no navegador do Windows):

| O quê | URL |
|---|---|
| Página (vídeo com as caixas do detector) | `http://localhost:8080/` |
| Página só com a câmera | `http://localhost:8080/?video_topic=/camera/image_raw` |
| Página na aba Coleta (vídeo em `/camera/image_raw`) | `http://localhost:8080/?tab=coleta` |
| Stream direto | `http://localhost:8081/stream?topic=/camera/image_raw&qos_profile=sensor_data` |
| Tópicos que o web_video_server enxerga | `http://localhost:8081/` |

O `qos_profile=sensor_data` é obrigatório no stream direto. A página já o inclui (`web/js/config.js`).

**Terminais:**

1. câmera, como na seção 2.1 (ou percepção completa, como na seção 2.2, para a página sem `?video_topic=`);
2. observação (`ros2 topic hz`, `ros2 topic info -v`);
3. web: `ros2 launch pantilt_web web.launch.py`;
4. hardware (só para o jog): `ros2 launch pantilt_bringup hardware.launch.py`.

Esperado no log do terminal 3: `Waiting For connections on 0.0.0.0:8081` (web_video_server) e `Rosbridge WebSocket server started on port 9090`.

Abra a página: `http://localhost:8080/?video_topic=/camera/image_raw`.

- O painel mostra `Aguardando imagens` e, em seguida, o vídeo.
- O código abaixo do painel mostra o tópico `/camera/image_raw`.
- O log do rosbridge mostra `Client connected`.
- No terminal 2, `ros2 topic info -v /camera/image_raw` mostra a assinatura do `web_video_server` como **BEST_EFFORT**.

**Testes de vídeo**

| # | Teste | Como | Esperado |
|---|---|---|---|
| 1 | Taxa com o navegador | página aberta; `ros2 topic hz /camera/image_raw` | mesma taxa da seção 2 |
| 2 | Duas abas | abra a página numa segunda aba | as duas mostram vídeo; `ros2 topic info -v` lista uma assinatura BEST_EFFORT por stream aberto; anote se a taxa cai |
| 3 | Latência visual | passe a mão rápido diante da câmera | anote o atraso percebido |
| 4 | Reiniciar a web | Ctrl+C no terminal 3 e suba o `web.launch.py` de novo | a página reconecta ao rosbridge e pede o stream de novo (`video.js`, `onConnect`); o vídeo volta sem recarregar |

**Testes com o hardware** (terminal 4 rodando; velocidades baixas no jog)

| # | Ação | Esperado |
|---|---|---|
| 5 | Segurar ► (pan positivo) | a imagem desliza no sentido do movimento; o indicador **Fonte** mostra `web` |
| 6 | Soltar o botão | o eixo para na hora (a página publica velocidade zero ao soltar) |
| 7 | Segurar ▲ (tilt positivo) | a imagem sobe ou desce de forma coerente |
| 8 | STOP com o eixo em movimento | parada imediata |
| 9 | Mover o pan até o limite | o eixo para perto de 29° e a imagem congela no enquadramento do limite |

### Preste atenção

- Se a assinatura do `web_video_server` aparecer como **RELIABLE**, o stream foi aberto sem `qos_profile=sensor_data` e não recebe quadros (seção 8).
- Anote o sentido da imagem em cada eixo nos testes 5 e 7. É isso que define o sinal do erro do `visual_servo_node` (`invert_pan`/`invert_tilt`).
- Com o sistema todo rodando, `df -h /dev/shm` deve mostrar algumas dezenas de MB usados, longe de 1 GB.

---

## 4. pantilt_control

### 4.1 scan_node

**O que valida:** o `scan_node`, servidor da action `/control/scan` (architecture.md §4.4). Ele faz uma varredura em zigue-zague: percorre o pan de uma ponta à outra (±28°) em cada faixa de tilt (-20°, 0°, 20°), um eixo por vez, e publica velocidades em `/ptu/cmd_vel_auto`. Só publica enquanto há um goal ativo.

**Pré-requisitos:** a seção 1.2 passou nos dois eixos, e o mecanismo foi zerado no centro mecânico (`/ptu/set_zero`). Deixe o STOP da página (ou o comando de parada) à mão.

**Terminal 1: hardware**

```bash
ros2 launch pantilt_bringup hardware.launch.py
```

**Terminal 2: controle**

```bash
ros2 launch pantilt_bringup control.launch.py
```

Esperado no log: `Pan [-28.0°, 28.0°] | faixas de tilt [-20.0, 0.0, 20.0]° | 15.0°/s | timeout 60 s | 20 Hz`.

**Terminal 3: observação**

```bash
ros2 action list -t                          # /control/scan [pantilt_interfaces/action/Scan]
ros2 topic echo /ptu/control_source          # auto durante a varredura
ros2 topic echo /ptu/cmd_vel_auto --field angular
ros2 topic echo /joint_states --field position
```

**Terminal 4: goals**

```bash
# goal padrão (0 = usa os parâmetros do nó)
ros2 action send_goal --feedback /control/scan pantilt_interfaces/action/Scan "{speed_deg_s: 0.0, timeout_s: 0.0}"
```

O `Ctrl+C` num `send_goal` ainda ativo cancela o goal.

| # | Teste | Comando / ação | Esperado |
|---|---|---|---|
| 1 | Varredura completa | goal padrão | log `Varredura iniciada em pan ..., tilt ...`; o pan vai primeiro à ponta mais próxima, depois percorre as três faixas alternando o sentido; feedback com `pass_index` de 0 a 2; resultado `completed: true`, `message: Padrão completo`; um zero em `/ptu/cmd_vel_auto` no fim |
| 2 | Velocidade própria | `"{speed_deg_s: 10.0, timeout_s: 0.0}"` | log com `10.0°/s`; varredura visivelmente mais lenta |
| 3 | Tempo esgotado | `"{speed_deg_s: 0.0, timeout_s: 5.0}"` | para após 5 s; resultado `completed: false`, `Tempo esgotado (5 s) antes do fim do padrão`; status SUCCEEDED (não é aborto) |
| 4 | Cancelamento | goal padrão e Ctrl+C no terminal 4 no meio da varredura | eixo para; log `Varredura cancelada`; status CANCELED |
| 5 | Goal substituído | goal padrão e, em outro terminal, um segundo goal padrão | log `Goal anterior substituído por um novo`; o primeiro termina ABORTED; a varredura recomeça da posição atual sem parar os eixos |
| 6 | Prioridade do operador | durante a varredura, segure um botão do jog na página | fonte `web` na hora e o eixo obedece ao jog; ao soltar, passado ~1 s, a fonte volta a `auto` e a varredura continua do ponto em que o eixo está, sem abortar |
| 7 | Goal recusado | `"{speed_deg_s: 40.0, timeout_s: 0.0}"` | CLI `Goal was rejected.`; log `Goal recusado: speed_deg_s=40.0 fora de [0, 30]°/s`; nada se move |
| 8 | Sem `/joint_states` | goal padrão e Ctrl+C no terminal 1 (derruba o hardware) | em ~0,5 s o log `Varredura abortada: Sem /joint_states há mais de 0.5 s`; status ABORTED; o eixo para (o bridge envia zero ao sair) |
| 9 | Ctrl+C no nó | suba o hardware de novo; goal padrão e Ctrl+C no terminal 2 | log `Varredura abortada: Nó encerrado`; zero em `/ptu/cmd_vel_auto`; o eixo para |
| 10 | Parâmetro inválido | com o `control.launch.py` parado: `ros2 run pantilt_control scan_node --ros-args -p speed_deg_s:=40.0` | `[FATAL] Parâmetro inválido: speed_deg_s deve ser no máximo 30°/s` e o nó sai |

Para registrar: `ros2 bag record /joint_states /ptu/cmd_vel_auto /ptu/cmd_vel /ptu/control_source`.

### Preste atenção

- **Nenhum eixo pode chegar ao batente.** Se isso acontecer, pare na hora e volte à seção 1.2: a causa mais provável é telemetria errada.
- Se nenhum eixo se mover por mais de 3 s com velocidade comandada, a varredura aborta com `Nenhum eixo se moveu em 3 s ...` (eixo travado, no limite do bridge ou com sentido de giro invertido). Esse aborto leva 3 s: fique com o STOP à mão.
- Perto de cada alvo a velocidade cai (até 2°/s); a ultrapassagem deve ficar abaixo de ~1°. Anote se passar disso.
- A varredura usa ±28° e o bridge limita em ±29°: o log `Limite de ângulo atingido` não deve aparecer. Se aparecer, a ultrapassagem está grande demais.

### 4.2 visual_servo_node

**O que valida:** o `visual_servo_node`, servidor da action `/control/center` (architecture.md §4.5). Ele converte o erro do alvo em ângulo (`θ = atan(e_px / f)`, com `f` do `/camera/camera_info`), passa por um PID por eixo e publica velocidades em `/ptu/cmd_vel_auto` a 20 Hz. No modo CENTER termina ao centralizar; no TRACK segue rastreando até ser cancelado.

**Pré-requisitos:**
- seção 1.2 aprovada nos dois eixos e mecanismo zerado no centro mecânico;
- `camera_intrinsics.yaml` presente: o `camera_node` registra `Calibração: fx=...` ao subir (seção 2.3);
- STOP da página à mão. **O primeiro movimento é com `kp = 0.2`**, porque os sinais (`invert_pan`/`invert_tilt`) ainda não foram fixados (tarefa E).

**Terminais** (cada um com a seção 0 aplicada):

```bash
ros2 launch pantilt_bringup hardware.launch.py           # 1
ros2 launch pantilt_bringup perception.launch.py         # 2
ros2 launch pantilt_web web.launch.py                    # 3 (vídeo, jog e STOP)
ros2 run pantilt_control visual_servo_node --ros-args \
  --params-file /ros2_ws/src/pantilt_ros/pantilt_bringup/config/params.yaml \
  -p pan.kp:=0.2 -p tilt.kp:=0.2                         # 4
```

Esperado no log do terminal 4: `Controlador pid | pan kp=0.2 ki=0.0 kd=0.0 | tilt kp=0.2 ... | máx 20.0°/s | invert pan=False tilt=False | 20 Hz`.

**Terminal 5: observação**

```bash
ros2 topic echo /ptu/cmd_vel_auto --field angular
ros2 topic echo /ptu/control_source
```

**Terminal 6: alvo e goals**

```bash
ros2 service call /perception/set_target pantilt_interfaces/srv/SetTarget "{equipment: 'garrafa'}"
# CENTER (mode 0) e TRACK (mode 1)
ros2 action send_goal -f /control/center pantilt_interfaces/action/Center \
  "{mode: 0, tolerance_px: 20.0, hold_time_s: 1.0, lost_timeout_s: 1.0}"
ros2 action send_goal -f /control/center pantilt_interfaces/action/Center \
  "{mode: 1, tolerance_px: 20.0, hold_time_s: 1.0, lost_timeout_s: 1.0}"
```

O `Ctrl+C` num `send_goal` ainda ativo cancela o goal.

| # | Teste | Comando / ação | Esperado |
|---|---|---|---|
| 1 | Sem calibração | `camera_node` sem o arquivo de intrínsecos (ou sem o terminal 2) e goal CENTER | CLI `Goal was rejected.`; log `Goal recusado: sem /camera/camera_info válido ...`; nada se move. Não precisa da ESP32 |
| 2 | Sentido de giro | `kp = 0.2`; alvo a ~100 px à direita do centro, depois abaixo; goal CENTER | o eixo leva o alvo para o centro. Se afastar, cancele na hora e reinicie o terminal 4 com `-p invert_pan:=true` (ou `invert_tilt`). Pela calibração (pan+ → alvo à esquerda, tilt+ → alvo para baixo), o esperado é `invert_pan=false` e `invert_tilt=true`. Anote o resultado |
| 3 | CENTER | sinais corrigidos, `kp` do `params.yaml` (0,8); alvo fora do centro; goal CENTER | log `Centralização iniciada (CENTER) \| f_x=... f_y=...`; resultado `success: true`, `message: centralizado em X s, erro residual Y px`, com `convergence_time_s` e `final_error_px` iguais a X e Y; um zero no fim |
| 4 | TRACK | goal TRACK e mova o alvo devagar | feedback com `centered` alternando conforme o erro; a action continua ativa; Ctrl+C no terminal 6 → CANCELED, `cancelado`, eixos param |
| 5 | Alvo perdido | goal TRACK e cubra o alvo | em ~1 s: ABORTED, `alvo perdido`; `/ptu/cmd_vel_auto` zera ~0,25 s depois de cobrir |
| 6 | Goal substituído | goal TRACK e, em outro terminal, um segundo goal TRACK | log `Goal anterior substituído por um novo`; o primeiro termina ABORTED (`substituído por um novo goal`); o rastreamento continua sem parar |
| 7 | Prioridade do operador | durante o TRACK, segure o jog | fonte `web` na hora e o eixo obedece ao jog; ao soltar, passado ~1 s, a fonte volta a `auto` e o servo retoma, sem abortar |
| 8 | Preso no limite | goal TRACK e leve o alvo para o lado até o pan parar no limite do bridge (o alvo ainda visível) | log do bridge `Limite de ângulo atingido`; 2 s depois, ABORTED com `pan preso no limite positivo (29.0°): comando empurrando há 2.0 s sem movimento` (ou negativo) |
| 9 | Ctrl+C no nó | goal TRACK e Ctrl+C no terminal 4 | log `Centralização abortada: nó encerrado`; zero em `/ptu/cmd_vel_auto`; os eixos param |
| 10 | Goal inválido | `"{mode: 0, tolerance_px: 0.0, hold_time_s: 1.0, lost_timeout_s: 1.0}"` | `Goal was rejected.`; log `Goal recusado: tolerance_px=0.0 deve ser positivo` |
| 11 | Parâmetro inválido | `ros2 run pantilt_control visual_servo_node --ros-args -p max_vel_deg_s:=40.0` | `[FATAL] Parâmetro inválido: max_vel_deg_s deve estar em (0, 30]°/s, recebido 40.0` e o nó sai |

Para registrar (gráficos do TCC e `e_d`): o `experiment_recorder` grava cada goal numa pasta própria, com os sinais internos da malha (seção 7.2). Alternativa crua: `ros2 bag record --include-hidden-topics /perception/target /ptu/cmd_vel_auto /joint_states /camera/camera_info /control/center/_action/feedback /control/servo_debug`.

### Preste atenção

- **Sinal invertido leva o eixo para longe do alvo.** No pan, o aborto por limite vem 2 s depois de o eixo parar no limite do bridge; no tilt (±89°) o eixo anda muito antes disso. Cancele ou use o STOP sem esperar o aborto.
- **Métricas:** `convergence_time_s` (t_c) vai da primeira detecção válida até o instante em que o erro entrou na tolerância para ficar `hold_time_s`; `final_error_px` (e_r) é a média do erro nessa janela. No TRACK e nos abortos, o resultado traz as métricas da primeira convergência (0 se não houve).
- **Perda:** o prazo conta do instante de captura do último quadro com alvo; com ~115 ms de atraso do detector, o aborto chega ~0,9 s depois do último alvo recebido.
- **`f` fixo por goal:** o nó lê `f_x` e `f_y` no início de cada goal. Depois de recalibrar, reinicie o `camera_node` e mande um goal novo.
- Um aviso `Alvo em WxH, mas o /camera/camera_info é de ...` indica calibração de outra resolução: refaça a 2.3.
- Sem `/joint_states`, o nó avisa `checagem de limite desligada` e segue; o bridge sem telemetria não deixa os eixos se moverem.
- Os sinais definitivos e os ganhos vão para o `params.yaml` na tarefa E.

---

## 5. pantilt_dataset

**O que valida:** o `capture_node` (architecture.md §4.9), que grava `/camera/image_raw` em MP4, com um `.json` de metadados ao lado, e opcionalmente mantém a varredura ativa com goals `Scan` em sequência; o `dataset.launch.py`, que sobe a coleta com um comando; e o painel de coleta da página (aba **Coleta**). Os vídeos vão para `/ros2_ws/datasets/` (no Windows: `ros2_ws\datasets`), com o nome `AAAAMMDD_HHMMSS_<sessao>.mp4`.

**Pré-requisitos:** a câmera funcionando (seção 2). Para os testes com varredura, também o hardware, com a seção 1.2 aprovada (como na seção 4). Confira o espaço livre antes: `df -h /ros2_ws`.

O `dataset.launch.py` sobe sempre o `camera_node` e o `capture_node`. Os argumentos ligam o resto:

| Argumento | Padrão | O que inclui |
|---|---|---|
| `hardware` | `true` | `hardware.launch.py` (bridge e `command_mux`) e `control.launch.py` (`scan_node`): jog e varredura |
| `web` | `true` | `web.launch.py` do `pantilt_web` (página, rosbridge e web_video_server) |
| `params_file` | `config/params.yaml` | arquivo de parâmetros de todos os nós |

O detector não sobe: a coleta não precisa da YOLO.

### 5.1 capture_node

Roteiro aprovado no hardware em 29/09/2026 (com `ros2 run`). Refaça só se o `capture_node` mudar.

**Terminal 1: launch** (sem hardware nem página)

```bash
ros2 launch pantilt_bringup dataset.launch.py hardware:=false web:=false
```

Esperado no log: `Fonte aberta: 640x480 @ 30.0 fps (MJPG)` (camera_node) e `Saída: /ros2_ws/datasets | 15.0 fps | fourcc "mp4v" | mínimo 1.0 GB livres | fila 30` (capture_node).

**Terminal 2: observação**

```bash
ros2 topic echo /capture/status             # transient_local: mostra o estado atual ao conectar; 2 Hz
ls -la /ros2_ws/datasets/
```

**Terminal 3: comandos**

```bash
# início sem varredura
ros2 service call /capture/start pantilt_interfaces/srv/StartCapture "{session: 'Mesa Janela (tarde)', scan: false, speed_deg_s: 0.0}"
# início com varredura (0 = velocidade do scan_node)
ros2 service call /capture/start pantilt_interfaces/srv/StartCapture "{session: 'mesa', scan: true, speed_deg_s: 0.0}"
# fim
ros2 service call /capture/stop std_srvs/srv/Trigger
```

**Testes sem hardware** (só a câmera; launch com `hardware:=false web:=false`)

| # | Teste | Comando / ação | Esperado |
|---|---|---|---|
| 1 | Gravação simples | início sem varredura, ~20 s, fim | `accepted=True` com `file` terminando em `_mesa_janela_tarde.mp4`; status com `recording: true`, `frames` subindo, `dropped: 0`, `message: Gravando`; no fim, `Gravado: N quadros em X s (F fps, M MB) -> <arquivo>`; MP4 e `.json` na pasta |
| 2 | Conteúdo do `.json` | `cat /ros2_ws/datasets/<nome>.json` | `frames`, `duration_s`, `fps_real` (~15), `width`/`height` 640×480, `size_bytes`, `scan.enabled: false`, `end_reason: Parada pelo operador` |
| 3 | Vídeo no Windows | abra o MP4 pelo Explorer | toca normalmente; anote o tamanho por minuto (`size_bytes` / `duration_s`) |
| 4 | Início duplicado | com uma gravação ativa, outro início | `accepted=False`, `Já existe uma gravação ativa (...)`; a gravação atual continua |
| 5 | Fim sem gravação | `/capture/stop` sem gravação ativa | `success=False`, `Nenhuma gravação ativa` |
| 6 | Câmera parou | durante a gravação: `pkill -INT -f lib/pantilt_perception/camera_node`; depois `ros2 run pantilt_perception camera_node --ros-args --params-file <params.yaml>` | em ~2 s, status `Sem quadros de /camera/image_raw há mais de 2 s (a câmera está rodando?)` e um aviso no log; a gravação continua; com a câmera de volta, o status volta a `Gravando` |
| 7 | Gravação sem quadros | câmera parada (como no teste 6); início e fim | `Nenhum quadro recebido; nada gravado`; nenhum arquivo criado |
| 8 | Varredura sem `scan_node` | início com `scan: true` (launch com `hardware:=false`) | `accepted=False`, `scan_node indisponível: suba o control.launch.py ou use scan=false` |
| 9 | Velocidade inválida | início com `speed_deg_s: 45.0` | `accepted=False`, `speed_deg_s=45.0 fora de [0, 30]°/s` |
| 10 | Ctrl+C no nó | durante a gravação, Ctrl+C no terminal 1 (encerra o launch) | log `Nó encerrado. Gravado: ...`; MP4 e `.json` completos, com `end_reason: Nó encerrado` |
| 11 | Pouco espaço | com o launch parado, uma câmera com `ros2 run` (como no teste 6) e `ros2 run pantilt_dataset capture_node --ros-args --params-file <params.yaml> -p min_free_gb:=10000.0`; depois um início | `accepted=False`, `Pouco espaço em disco: X GB livres (mínimo 10000.0 GB)` |
| 12 | Parâmetro inválido | `ros2 run pantilt_dataset capture_node --ros-args -p fourcc:=mp4` | `[FATAL] Parâmetro inválido: fourcc="mp4" inválido ...` e o nó sai |

**Testes com varredura.** Suba o launch com o hardware (`ros2 launch pantilt_bringup dataset.launch.py web:=false`, ou com a página para o teste 14). Mecanismo zerado no centro; STOP à mão. No teste 16, que derruba só o `scan_node`, use o `pkill` indicado.

| # | Teste | Comando / ação | Esperado |
|---|---|---|---|
| 13 | Gravação com varredura | início com `scan: true` | status `scanning: true`, `Gravando com varredura (passada 1)`; o `scan_node` registra `Varredura iniciada`; ao fim do padrão, `Varredura terminada: Padrão completo` e logo outra `Varredura iniciada` (passada 2), sem parar a gravação |
| 14 | Jog durante a gravação | segure um botão do jog na página | fonte `web`, o eixo obedece ao jog; a gravação não para; ao soltar, a varredura continua (como no teste 6 da seção 4) |
| 15 | Fim durante a varredura | `/capture/stop` com a varredura ativa | o `scan_node` registra `Varredura cancelada` e os eixos param; `.json` com `scan.goals_sent`/`goals_succeeded` |
| 16 | Varredura interrompida | durante a gravação com varredura: `pkill -INT -f lib/pantilt_control/scan_node` | status `scanning: false`, `Gravando sem varredura: interrompida, ...` (ex.: `scan_node saiu do ar`); a gravação continua; nenhum goal novo é enviado |
| 17 | Ctrl+C durante a varredura | reinicie o launch, início com varredura e Ctrl+C no terminal 1 (encerra o launch) | o `scan_node` registra `Varredura cancelada`; arquivo finalizado com `end_reason: Nó encerrado` |

### 5.2 dataset.launch.py

**O que valida:** que um único comando sobe a coleta inteira e que um único Ctrl+C a encerra com o arquivo finalizado e os eixos parados.

| # | Teste | Comando / ação | Esperado |
|---|---|---|---|
| 1 | Argumentos | `ros2 launch pantilt_bringup dataset.launch.py --show-args` | lista `params_file`, `hardware` e `web` (e as portas do `web.launch.py`) |
| 2 | Só a coleta | `ros2 launch pantilt_bringup dataset.launch.py hardware:=false web:=false`; `ros2 node list` | `/camera_node` e `/capture_node` |
| 3 | Coleta + página | `... hardware:=false`; `ros2 node list` | os de cima + `/rosbridge_websocket` e `/web_video_server`; log `Waiting For connections on 0.0.0.0:8081` e `Rosbridge WebSocket server started on port 9090` |
| 4 | Tudo | `ros2 launch pantilt_bringup dataset.launch.py`; `ros2 node list` | os de cima + `/serial_bridge_node`, `/command_mux` e `/scan_node`; log `Conectado a /dev/ttyUSB0 ...` e `Pan [-28.0°, 28.0°] \| faixas de tilt ...` |
| 5 | Varredura sem hardware | launch do teste 3; início com `scan: true` (página ou CLI) | `accepted=False`, `scan_node indisponível: suba o control.launch.py ou use scan=false` |
| 6 | Ctrl+C único sem varredura | launch do teste 3; gravação sem varredura por ~10 s; Ctrl+C no terminal do launch | log `Nó encerrado. Gravado: ...`; todos os processos com `process has finished cleanly`; `.json` com `end_reason: Nó encerrado` |
| 7 | Ctrl+C único com varredura | launch do teste 4; gravação com varredura; Ctrl+C no meio de uma passada | os eixos param (o bridge envia zero ao sair); MP4 e `.json` finalizados com `end_reason: Nó encerrado` |
| 8 | Parâmetros próprios | `cp` do `params.yaml` (seção 0) com `output_dir: "/tmp/ds"`; `... params_file:=/tmp/params_teste.yaml` | o log do capture_node mostra `Saída: /tmp/ds`; o vídeo vai para lá |

No teste 6 (sem hardware), o encerramento foi conferido no desenvolvimento (02/10/2026): um vídeo de 61 s foi finalizado no Ctrl+C. O teste 7 é o que importa no hardware.

### 5.3 Painel de coleta

**O que valida:** a aba **Coleta** da página, que chama `/capture/start` e `/capture/stop` e mostra o `/capture/status`.

**Terminal 1:** `ros2 launch pantilt_bringup dataset.launch.py` (tudo). **Terminal 2:** `ros2 topic echo /capture/status` e `ls -la /ros2_ws/datasets/`.

Abra `http://localhost:8080/?tab=coleta`. O painel tem:

- o nome da sessão;
- **Varredura** (marcada por padrão) e a velocidade em °/s (0 = padrão do `scan_node`, 15°/s);
- **Gravar** e **Parar**;
- o estado, com o tempo, os quadros e o arquivo.

| # | Teste | Como | Esperado |
|---|---|---|---|
| 1 | Abertura | página com `?tab=coleta` | aba Coleta selecionada; vídeo de `/camera/image_raw` (o código sob o placeholder mostra esse tópico); estado `Parado`; nota do card vazia |
| 2 | Troca de aba | clique em Inspeção e depois em Coleta | na Inspeção o vídeo passa a `/perception/debug_image` e fica em `Aguardando imagens` (esperado sem o detector); na Coleta volta a câmera |
| 3 | Sessão vazia | Gravar sem nome | o navegador pede o campo; nada é chamado |
| 4 | Gravação sem varredura | desmarque Varredura, sessão `teste painel`, Gravar; ~20 s; Parar | toast `Gravando sem varredura em ...`; estado `Gravando` (vermelho), tempo e quadros subindo; chip `● REC mm:ss` no canto do vídeo e `●` na aba; os campos ficam bloqueados. No Parar: toast `Gravado: N quadros ...`; estado `Parado`; a mensagem mostra `Parada pelo operador. Gravado: ...`; o chip some |
| 5 | Gravação com varredura | marque Varredura, velocidade 0, Gravar | estado `Gravando + varredura` (laranja); mensagem `Gravando com varredura (passada 1)`; **Fonte** = `auto`; o pan-tilt varre e a imagem acompanha |
| 6 | Velocidade própria | Parar; velocidade 10, Gravar | varredura visivelmente mais lenta; log do scan_node com `10.0°/s` |
| 7 | Jog durante a gravação | segure um botão do jog | **Fonte** = `web`, o eixo obedece; a gravação continua; ao soltar, ~1 s depois, a varredura continua |
| 8 | Parar durante a varredura | Parar no painel | eixos param; scan_node `Varredura cancelada`; `.json` com `scan.goals_sent` ≥ 1 |
| 9 | "Parar tudo" durante a varredura | Gravar com varredura; clique em **Parar tudo** | o eixo para; ~1 s depois a varredura **volta** (comportamento esperado, ver "Preste atenção"); a gravação continua |
| 10 | Recusa | Parar; `pkill -INT -f lib/pantilt_control/scan_node`; Gravar com Varredura | toast e linha de mensagem em vermelho `Recusado: scan_node indisponível ...` por ~8 s; nada gravado |
| 11 | Recarregar a página | durante uma gravação, F5 | o painel volta direto em `Gravando`, com o tempo correto (o status é transient_local) |
| 12 | Duas abas do navegador | abra a página em outra aba; Parar numa delas | as duas mostram o mesmo estado |
| 13 | capture_node fora do ar | durante uma gravação: `pkill -INT -f lib/pantilt_dataset/capture_node` | arquivo finalizado (`Nó encerrado`); em ~2 s a nota do card fica `capture_node indisponível (sem /capture/status)` e Gravar/Parar ficam desabilitados; o chip REC some |
| 14 | capture_node de volta | `ros2 run pantilt_dataset capture_node --ros-args --params-file /ros2_ws/src/pantilt_ros/pantilt_bringup/config/params.yaml` | em poucos segundos o painel volta (estado `Parado`) sem recarregar a página |
| 15 | Web reiniciada | launch com `web:=false` e, em outro terminal, `ros2 launch pantilt_web web.launch.py`; durante uma gravação, Ctrl+C na web e suba de novo | a nota mostra `Sem conexão com o rosbridge`; ao reconectar, o painel volta em `Gravando` sem recarregar; a gravação não é afetada |

### 5.4 Sessão de coleta

Roteiro curto para gravar o dataset de verdade:

1. `df -h /ros2_ws`: anote o espaço livre. Use o tamanho por minuto medido no teste 3 da seção 5.1 para saber quantos minutos cabem.
2. Leve o mecanismo ao centro mecânico e zere (`Zerar eixos` na página).
3. `ros2 launch pantilt_bringup dataset.launch.py` (ou, equivalente, `ros2 launch pantilt_bringup system.launch.py mode:=dataset`).
4. Abra `http://localhost:8080/?tab=coleta`.
5. Para cada cena (posição do pan-tilt na sala):
   - sessão com o nome da cena e da luz (ex.: `mesa janela tarde`);
   - Varredura marcada e Gravar;
   - deixe completar pelo menos uma passada (`passada 2` na mensagem);
   - Parar.
6. Confira em cada gravação: `perd.` (descartados) ausente nos quadros, e `fps_real` do `.json` perto de 15.
7. Ao fim, Ctrl+C no launch e copie os vídeos de `ros2_ws\datasets` para fora do `C:` se o espaço apertar.

### Preste atenção

- **Disco:** o `/ros2_ws` fica no `C:` do Windows, com pouco espaço livre. O nó recusa o início e encerra a gravação (com o arquivo finalizado) abaixo de `min_free_gb`, verificando a cada 5 s. Anote no teste 3 da seção 5.1 quanto ocupa cada minuto, para planejar a coleta.
- **`dropped` > 0** (no painel, `N · M perd.` em laranja) significa que o disco ou a codificação não acompanham a câmera. Anote se aparecer; um `queue_size` maior só adia o problema.
- **Vídeo acelerado:** o MP4 declara `record_fps` (15). Se a câmera entregar menos (pouca luz), o vídeo toca mais rápido que a cena real. O `fps_real` do `.json` mostra a taxa gravada. Para extrair quadros para o dataset, isso não importa.
- **"Parar tudo" e STOP não encerram a varredura.** Eles publicam velocidade zero pela web; o `command_mux` dá prioridade ao operador por 1 s e depois a varredura volta. Para parar de varrer, use **Parar** no painel (ou `/capture/stop`).
- Uma varredura interrompida **não é retomada** sozinha: para voltar a varrer, encerre e inicie de novo com Varredura marcada.
- Se o `/capture/status` parar de chegar, o `capture_node` travou ou saiu: o painel mostra `capture_node indisponível`. Confira o terminal do launch.
- Com tudo num launch só, o log mistura os nós. Filtre com `grep`, ex.: `ros2 launch ... 2>&1 | grep -E 'capture_node|scan_node'`.
- O `camera_node` pode imprimir `Corrupt JPEG data: premature end of data segment` de vez em quando. É aviso do decodificador MJPG da câmera e não interrompe a gravação; anote se vier junto com queda de taxa.

---

## 6. pantilt_manager

### 6.1 inspection_manager

**O que valida:** o `inspection_manager` (architecture.md §4.3 e §6), que orquestra a inspeção: aplica o filtro no detector, varre com o `scan_node`, confirma o alvo em `confirm_frames` quadros consecutivos, centraliza com o `visual_servo_node` e volta ao IDLE. Aqui o teste é pela linha de comando; a aba Inspeção da página é o roteiro seguinte.

**Pré-requisitos:**
- seção 1.2 aprovada nos dois eixos e mecanismo zerado no centro mecânico;
- seções 2.2 e 2.3: detector com o `equipment_coco_test.yaml` e `camera_intrinsics.yaml` presente;
- sinais do servo conferidos (seção 4.2, testes 2 e 3). Os ganhos ainda não foram sintonizados (tarefa E);
- uma garrafa no alcance da varredura (pan de −28° a 28°, faixas de tilt −20°, 0° e 20°), fora do centro da imagem;
- STOP da página à mão. **O STOP e qualquer jog abortam a inspeção** (a fonte vira `web`).

**Terminais** (cada um com a seção 0 aplicada). Os terminais 1 a 5 podem ser trocados por um só, `ros2 launch pantilt_bringup system.launch.py` (seção 6.2); com eles separados, dá para derrubar uma camada sozinha (testes 2, 3, 13 e 14):

```bash
ros2 launch pantilt_bringup hardware.launch.py           # 1
ros2 launch pantilt_bringup perception.launch.py         # 2
ros2 launch pantilt_bringup control.launch.py            # 3 (scan_node + visual_servo_node)
ros2 launch pantilt_web web.launch.py                    # 4 (vídeo, jog e STOP)
ros2 run pantilt_manager inspection_manager --ros-args \
  --params-file /ros2_ws/src/pantilt_ros/pantilt_bringup/config/params.yaml   # 5
```

Esperado no log do terminal 5: `Equipamentos de .../equipment_coco_test.yaml: garrafa, copo, celular, tv | confirmação em 3 quadros | varredura até 60 s | até 2 readquisições | Center: 20.0 px por 1.0 s, perda em 1.0 s`. As transições aparecem no mesmo log (ex.: `SEARCHING -> CENTERING: Centralizando`).

**Terminal 6: observação**

```bash
ros2 topic echo /inspection/status
```

**Terminal 7: comandos**

```bash
ros2 service call /inspection/list_equipment pantilt_interfaces/srv/ListEquipment
# mode 0 = CENTER, 1 = TRACK
ros2 service call /inspection/start pantilt_interfaces/srv/StartInspection "{equipment: 'garrafa', mode: 0}"
ros2 service call /inspection/start pantilt_interfaces/srv/StartInspection "{equipment: 'garrafa', mode: 1}"
ros2 service call /inspection/stop std_srvs/srv/Trigger
```

| # | Teste | Comando / ação | Esperado |
|---|---|---|---|
| 1 | Lista | `list_equipment` | `keys` e `labels` do `equipment_coco_test.yaml`, na ordem do arquivo |
| 2 | Servidores fora do ar | terminal 3 parado; start CENTER | `accepted: false`, `Indisponível: scan_node (/control/scan): suba o control.launch.py; visual_servo_node ...`. Não precisa da ESP32 |
| 3 | Detector fora do ar | terminal 2 parado; start CENTER | `accepted: false`, `Indisponível: detector_node (/perception/set_target): suba o perception.launch.py` |
| 4 | Chave desconhecida | `{equipment: 'pneu', mode: 0}` | `accepted: false`, `Equipamento "pneu" desconhecido. Disponíveis: ...` |
| 5 | Operador recente | segure um jog e, em menos de 1 s depois de soltar, start CENTER | `accepted: false`, `Operador no controle ...`; 1 s depois, o mesmo start é aceito |
| 6 | CENTER | start CENTER | `Inspeção iniciada: Garrafa (teste COCO) (CENTER)`; status SEARCHING (`autonomous: true`) → CENTERING ao achar a garrafa → IDLE com `Centralizado em X s, erro residual Y px`. Log do detector: `Filtro: ...` no início e `Filtro removido` no fim |
| 7 | Passada sem alvo | tire a garrafa de cena; start CENTER | a varredura completa (~20 s com os padrões) e o status volta ao IDLE com `Alvo não encontrado (varredura completa)` |
| 8 | TRACK + stop | start TRACK; espere TRACKING; `stop` | `success: true`, IDLE com `Inspeção interrompida (/inspection/stop)`; log do servo `Centralização cancelada`; eixos param |
| 9 | Perda e readquisição | start TRACK; em TRACKING, cubra a garrafa por ~2 s e descubra | ~1 s depois de cobrir: SEARCHING com `Alvo perdido: nova varredura (readquisição 1/2)`; ao descobrir: CENTERING `Centralizando (readquisição 1/2)` → TRACKING |
| 10 | Sem readquisição | terminal 5 com `-p max_reacquire:=0`; start TRACK; cubra a garrafa | IDLE com `Alvo perdido 1 vezes seguidas (máximo de readquisições: 0)` |
| 11 | Operador assume | durante a inspeção, um jog (ou o STOP) | na hora: IDLE com `Inspeção abortada pelo operador`; o eixo obedece ao jog; goals cancelados e filtro removido |
| 12 | Start em andamento | start de novo durante a inspeção | `accepted: false`, `Inspeção em andamento (...)` |
| 13 | Ctrl+C no gerenciador | em TRACKING, Ctrl+C no terminal 5 | status IDLE com `Nó encerrado`; log do servo `Centralização cancelada`; `Filtro removido` no detector; eixos param |
| 14 | `scan_node` cai | durante a SEARCHING, Ctrl+C no terminal 3 | IDLE em até ~0,2 s, com `scan_node (/control/scan) saiu do ar` ou `Varredura interrompida: ...` |
| 15 | Stop em IDLE | `stop` sem inspeção | `success: false`, `Nenhuma inspeção ativa` |
| 16 | Parâmetro inválido | `ros2 run pantilt_manager inspection_manager --ros-args -p confirm_frames:=0` | `[FATAL] Parâmetro inválido: confirm_frames=0 inválido: deve ser pelo menos 1` e o nó sai |

### Preste atenção

- **Confirmação:** só contam alvos com `header.stamp` posterior à resposta do `set_target` e com a chave da inspeção; um `detected=false` zera a contagem. Com o detector a ~10 Hz, `confirm_frames: 3` leva ~0,3 s, e a varredura anda ~4,5° nesse tempo.
- **Readquisição:** só a perda do alvo (`alvo perdido`) leva a nova varredura. A contagem é de perdas **seguidas**: zera a cada vez que o alvo é centralizado. Outros abortos do servo (preso no limite, goal recusado) voltam ao IDLE com o motivo.
- **Uma passada por varredura:** a SEARCHING termina ao fim do padrão do `scan_node`, mesmo antes de `scan_timeout_s`.
- O `equipment_file` do gerenciador precisa ser o mesmo do `detector_node`. Uma chave que só um dos dois conhece é recusada com a mensagem do detector.
- O `error_px` do status vem do feedback da `Center` e é atualizado a 5 Hz.
- Para os gráficos, use o `experiment_recorder` (seção 7.2): uma pasta por inspeção, com os eventos do `/inspection/status`.

### 6.2 system.launch.py

**O que valida:** que um único comando sobe o sistema completo (inspeção) ou a coleta de vídeos, e que um único Ctrl+C encerra tudo com os eixos parados.

| Argumento | Padrão | O que faz |
|---|---|---|
| `mode` | `inspection` | `inspection`: `hardware.launch.py`, `perception.launch.py`, `control.launch.py`, `inspection_manager` e `web.launch.py`. `dataset`: o `dataset.launch.py` (seção 5.2), sem a YOLO |
| `hardware` | `true` | bridge e `command_mux`. Na inspeção, o controle sobe mesmo com `false` (os comandos só não chegam ao eixo); na coleta, `false` tira também o `scan_node`, como no `dataset.launch.py` |
| `web` | `true` | página, rosbridge e web_video_server (portas em `--show-args`) |
| `params_file` | `config/params.yaml` | arquivo de parâmetros de todos os nós |

```bash
ros2 launch pantilt_bringup system.launch.py                    # inspeção: http://localhost:8080/
ros2 launch pantilt_bringup system.launch.py mode:=dataset      # coleta:   http://localhost:8080/?tab=coleta
```

Os pré-requisitos da inspeção são os da seção 6.1; os da coleta, os da seção 5. Observação e comandos por CLI: terminais 6 e 7 da seção 6.1.

| # | Teste | Comando / ação | Esperado |
|---|---|---|---|
| 1 | Argumentos | `ros2 launch pantilt_bringup system.launch.py --show-args` | `params_file`, `mode` (com `Valid choices are: ['inspection', 'dataset']`), `hardware`, `web` e as portas do `web.launch.py` |
| 2 | Modo inválido | `... mode:=xyz` | `Argument "mode" provided value "xyz" is not valid` e nada sobe |
| 3 | Inspeção sem ESP32 e sem página | `... hardware:=false web:=false`; `ros2 node list` | log `Sistema em modo inspeção. Página: http://localhost:8080/`; `/camera_node`, `/detector_node`, `/scan_node`, `/visual_servo_node` e `/inspection_manager`; `list_equipment` responde |
| 4 | Inspeção completa | `ros2 launch pantilt_bringup system.launch.py`; `ros2 node list` | os de cima + `/serial_bridge_node`, `/command_mux`, `/rosbridge_websocket` e `/web_video_server`; logs `Conectado a /dev/ttyUSB0 ...`, `Rosbridge WebSocket server started on port 9090` e `Equipamentos de ...` (gerenciador) |
| 5 | Início antes do detector | logo depois de subir, start CENTER (página ou CLI) | `accepted: false`, `Indisponível: detector_node ...`; depois de `Equipamentos disponíveis` no log do detector, o mesmo start é aceito |
| 6 | Inspeção pela página | aba Inspeção, garrafa, CENTER | como o teste 6 da seção 6.1: SEARCHING → CENTERING → IDLE com `Centralizado em X s ...` |
| 7 | Ctrl+C único na inspeção | em TRACKING, Ctrl+C no terminal do launch | eixos param; o terminal 6 mostra IDLE com `Nó encerrado`; o launch volta ao prompt; a linha `ps aux` da seção 0 não lista nenhum nó |
| 8 | Coleta | `... mode:=dataset`; `ros2 node list` | log `Sistema em modo coleta. Página: http://localhost:8080/?tab=coleta`; os nós do teste 4 da seção 5.2 (sem `/detector_node`, `/visual_servo_node` e `/inspection_manager`) |
| 9 | Coleta com varredura e Ctrl+C | página com `?tab=coleta`, Gravar com Varredura; Ctrl+C no meio de uma passada | como o teste 7 da seção 5.2: eixos param; MP4 e `.json` com `end_reason: Nó encerrado` |
| 10 | Parâmetros próprios | `cp` do `params.yaml` (seção 0) com `output_dir: "/tmp/ds"`; `... mode:=dataset params_file:=/tmp/params_teste.yaml` | log do capture_node `Saída: /tmp/ds` |

No desenvolvimento (15/10/2026, sem ESP32 e com um `.mp4` como câmera), os testes 1 a 3 e 8 passaram, e uma gravação com `mode:=dataset hardware:=false web:=false` foi finalizada pelo Ctrl+C (`end_reason: Nó encerrado`). Os testes 4 a 7, 9 e 10 ficam para o hardware.

### Preste atenção

- **Um launch com câmera por vez.** Não suba o `system.launch.py` junto do `dataset.launch.py`, do `perception.launch.py` ou do `hardware.launch.py`: dois `camera_node` disputam o `/dev/video0` e dois bridges, a serial.
- **Detector lento para subir:** no volume 9p, o `import` do torch leva de 1 a 2 min com os outros nós subindo junto (65 s sozinho, 118 s no `system`, medidos no desenvolvimento). Até o `Equipamentos disponíveis` no log do detector, o `/inspection/start` é recusado.
- **Ctrl+C:** alguns nós aparecem como `process has died [... exit code -2 ...]`, com traceback no `destroy_node`. É o segundo SIGINT que o `ros2 launch` repassa durante o encerramento (seção 0). Acontece igual no `perception.launch.py` e no `control.launch.py`. Um Ctrl+C durante a carga do detector leva o launch a escalar para SIGTERM depois de 5 s; espere o detector subir antes de encerrar.
- **Log misturado:** filtre com `grep`, ex.: `ros2 launch pantilt_bringup system.launch.py 2>&1 | grep -E 'inspection_manager|visual_servo'`.
- Para reiniciar uma camada sozinha (ex.: a web, teste 15 da seção 5.3), suba com `web:=false` e rode o `web.launch.py` em outro terminal.

---

## 7. pantilt_experiment

Aquisição de dados para os ensaios do TCC (architecture.md §4.11). Foi feita em etapas, e cada uma tem o seu roteiro. Se algo quebrar, refaça na ordem: cada seção pressupõe as anteriores.

| Etapa | O que entrou | Roteiro |
|---|---|---|
| 1 | interfaces novas, `/control/servo_debug` (θ, P/I/D, saturação, comando) e `/perception/timing` (inferência e atraso por quadro) | 7.1 |
| 2 | `experiment_recorder`: uma pasta por inspeção ou goal `Center` direto, com CSVs e `resumo.json` | 7.2 |
| 3 | imagens: snapshots ou vídeo | 7.3 |
| 4 | bloco Ensaio na aba Inspeção da página | 7.4 |
| 5 | `analisar_ensaios`: métricas e gráficos | 7.5 |

### 7.1 Sinais de diagnóstico (etapa 1)

**O que valida:** as interfaces novas do `pantilt_interfaces`, o `/control/servo_debug` do `visual_servo_node` (architecture.md §4.5, "Diagnóstico") e o `/perception/timing` do `detector_node` (§4.2). Nenhum comportamento da malha muda: os testes da seção 4.2 continuam valendo.

**Pré-requisitos:** seções 2.2, 2.3 e 4.2 aprovadas (detector, calibração e sinais do servo). O teste 1 e o 2 dispensam hardware e câmera.

**Build e testes automáticos (terminal 1):**

```bash
cd /ros2_ws && colcon build --symlink-install \
  --packages-select pantilt_interfaces pantilt_control pantilt_perception && ws
ros2 interface show pantilt_interfaces/msg/ServoDebug
ros2 interface show pantilt_interfaces/msg/DetectionTiming
cd /ros2_ws/src/pantilt_ros/pantilt_control && python3 -m pytest -q test && cd /ros2_ws
```

**Terminais** (cada um com a seção 0 aplicada). O servo fica num terminal próprio para os testes 9 e 10 o reiniciarem com outro parâmetro:

```bash
ros2 launch pantilt_bringup hardware.launch.py           # 2
ros2 launch pantilt_bringup perception.launch.py         # 3
ros2 launch pantilt_web web.launch.py                    # 4 (vídeo, jog e STOP)
ros2 run pantilt_control visual_servo_node --ros-args \
  --params-file /ros2_ws/src/pantilt_ros/pantilt_bringup/config/params.yaml   # 5
ros2 topic echo /control/servo_debug                     # 6
ros2 topic echo /perception/timing                       # 7
```

**Terminal 8: comandos.** Use uma chave do `equipment_file` em uso (`ros2 service call /inspection/list_equipment pantilt_interfaces/srv/ListEquipment`); abaixo, `garrafa`.

```bash
ros2 service call /perception/set_target pantilt_interfaces/srv/SetTarget "{equipment: 'garrafa'}"
ros2 action send_goal -f /control/center pantilt_interfaces/action/Center \
  "{mode: 0, tolerance_px: 20.0, hold_time_s: 1.0, lost_timeout_s: 1.0}"
ros2 action send_goal -f /control/center pantilt_interfaces/action/Center \
  "{mode: 1, tolerance_px: 20.0, hold_time_s: 1.0, lost_timeout_s: 1.0}"
```

| # | Teste | Comando / ação | Esperado |
|---|---|---|---|
| 1 | Interfaces | `ros2 interface show` (terminal 1), também para `msg/ExperimentStatus` e `srv/ConfigureExperiment` | campos como na architecture.md §4.5 e §4.11 |
| 2 | pytest | terminal 1 | todos passam, inclusive `test_termos_somam_a_saida_bruta`, `test_saida_bruta_acima_da_saturacao`, `test_reset_zera_termos` e `test_padroes_da_interface` |
| 3 | Taxa do timing | `ros2 topic hz /perception/timing`, sem filtro e sem inspeção | a taxa do detector (~10 Hz a `imgsz` 640), mesmo sem filtro |
| 4 | Valores do timing | terminal 7 | `inference_ms` ~70–95 e `latency_ms` ~110–120 (as médias do log de 10 s do detector); `n_detections` = caixas na cena |
| 5 | Sem goal | `ros2 topic hz /control/servo_debug` | nada publicado |
| 6 | CENTER | `set_target` e goal CENTER com a garrafa fora do centro | uma mensagem por alvo (~10 Hz) com `controller: pid` e `mode: 0`. Com `ki = kd = 0`: `i_*` e `d_*` zerados, `p_pan` = `kp · theta_pan`, `u_raw_pan` = `p_pan + i_pan + d_pan`, `theta_pan` ≈ `atan(error_x / f_x)`. Sem saturação: `cmd_pan` = `u_raw_pan` e `cmd_tilt` = −`u_raw_tilt` (`invert_tilt: true`) |
| 7 | Métricas | fim do goal do teste 6 | a última mensagem tem `centered: true`, e `convergence_time_s` e `final_error_px` iguais aos do resultado da action |
| 8 | Alvo coberto | goal TRACK e cubra a garrafa | `target_visible: false`, `error_px: 0.0`, ângulos, termos e `cmd_*` zerados, até o aborto por `alvo perdido` |
| 9 | Saturação | reinicie o terminal 5 com `-p max_vel_deg_s:=2.0` no fim; goal CENTER com a garrafa longe do centro | `saturated_pan: true` enquanto \|`u_raw_pan`\| > 0,0349 rad/s (2°/s), com `cmd_pan` preso em ±0,0349 |
| 10 | Diagnóstico desligado | reinicie o terminal 5 com `-p publish_debug:=false`; goal CENTER | nada em `/control/servo_debug`; a centralização funciona igual |
| 11 | Timing desligado | troque o terminal 3 por `ros2 run pantilt_perception camera_node` e `ros2 run pantilt_perception detector_node`, ambos com `--ros-args --params-file .../params.yaml`, e o detector com `-p publish_timing:=false` | `ros2 topic info /perception/timing` mostra `Publisher count: 0`; o log de 10 s continua |

**Preste atenção:**

- O `/control/servo_debug` só sai com goal `Center` ativo e só quando chega um alvo novo (taxa do detector, não os 20 Hz do comando). O comando de fato enviado continua em `/ptu/cmd_vel_auto`.
- `u_raw_*` é a saída **antes** da saturação e **sem** o `invert_*`; `cmd_*` é depois dos dois. Por isso o sinal de `cmd_tilt` é o oposto de `u_raw_tilt` com `invert_tilt: true`.
- `dt` é 0 na primeira amostra do goal e depois de uma lacuna (`gap_reset: true`, alvo sumido por mais de 0,5 s).
- O `latency_ms` usa o relógio do host contra o `header.stamp` da imagem, o mesmo cálculo das médias do log.

### 7.2 experiment_recorder (etapa 2)

**O que valida:** o `experiment_recorder` (architecture.md §4.11): abre uma pasta quando a inspeção sai do IDLE ou quando chega um goal `Center` direto, grava os sinais em CSV, fecha depois da cauda com o `resumo.json` e renomeia a pasta com o equipamento e o modo.

**Pré-requisitos:** seções 6.1 (inspeção completa) e 7.1 aprovadas. Os testes 1 e 16 dispensam hardware e câmera; os demais precisam do sistema de inspeção funcionando.

**Build e testes automáticos (terminal 1):**

```bash
cd /ros2_ws && colcon build --symlink-install \
  --packages-select pantilt_interfaces pantilt_experiment pantilt_bringup && ws
cd /ros2_ws/src/pantilt_ros/pantilt_experiment && python3 -m pytest -q test && cd /ros2_ws
```

**Terminais** (cada um com a seção 0 aplicada). O `system.launch.py` já sobe o gravador; aqui ele fica num terminal próprio para os testes de Ctrl+C:

```bash
ros2 launch pantilt_bringup system.launch.py recorder:=false   # 2
ros2 run pantilt_experiment experiment_recorder --ros-args \
  --params-file /ros2_ws/src/pantilt_ros/pantilt_bringup/config/params.yaml   # 3
```

Esperado no log do terminal 3: `Saída: /ros2_ws/ensaios | rótulo "sem_rotulo" (próxima #001) | imagens: none | pré 1.0 s, cauda 1.0 s | mínimo 1.0 GB livres | commit 1a2b3c4d`.

**Terminal 4: observação**

```bash
ros2 topic echo /experiment/status
```

**Terminal 5: comandos.** Troque `garrafa` por uma chave do `equipment_file` em uso.

```bash
ros2 service call /experiment/configure pantilt_interfaces/srv/ConfigureExperiment \
  "{label: 'teste_gravador', notes: 'roteiro 7.2', image_mode: 'none'}"
ros2 service call /inspection/start pantilt_interfaces/srv/StartInspection "{equipment: 'garrafa', mode: 0}"
ros2 service call /inspection/start pantilt_interfaces/srv/StartInspection "{equipment: 'garrafa', mode: 1}"
ros2 service call /inspection/stop std_srvs/srv/Trigger
# goal direto (gerenciador em IDLE)
ros2 service call /perception/set_target pantilt_interfaces/srv/SetTarget "{equipment: 'garrafa'}"
ros2 action send_goal /control/center pantilt_interfaces/action/Center \
  "{mode: 0, tolerance_px: 20.0, hold_time_s: 1.0, lost_timeout_s: 1.0}"
# última pasta gravada
U=$(ls -td /ros2_ws/ensaios/teste_gravador/*/ | head -1); echo $U; ls $U
python3 -m json.tool $U/resumo.json | less
cat $U/events.csv
```

| # | Teste | Comando / ação | Esperado |
|---|---|---|---|
| 1 | pytest | terminal 1 | todos passam (`test_session.py` e `test_csv_writer.py`) |
| 2 | Configuração | `configure` com `teste_gravador` | `success: true`, `Próxima gravação: teste_gravador #001 \| imagens: none \| obs.: roteiro 7.2`; status com `label: teste_gravador`, `repetition: 1`, `recording: false` |
| 3 | Inspeção CENTER | start CENTER | status `recording: true`, `Gravando inspeção: teste_gravador #001`; ao voltar ao IDLE, `(cauda)` por ~1 s e então `Gravado: teste_gravador/001_AAAAMMDD_HHMMSS_garrafa_center (centralizado, X s)`. Log do terminal 3 com `Gravação iniciada (inspeção): ...` e `Gravado: ...` |
| 4 | Arquivos | `ls $U` | `target.csv`, `detections.csv`, `timing.csv`, `servo.csv`, `center_feedback.csv`, `scan_feedback.csv`, `joints.csv`, `cmd_auto.csv`, `cmd_mux.csv`, `events.csv` e `resumo.json`. `cmd_mux.csv` só aparece com o hardware (o `command_mux`) de pé |
| 5 | Resumo | `json.tool` | `desfecho: centralizado`; `center[0]` com `status: sucesso`, `success: true` e `convergence_time_s`/`final_error_px` iguais à mensagem final; `scan[0]` com `status: cancelado`; `parametros` com os 4 nós (nenhum `ausente`), `pan.kp` igual ao `params.yaml`; `camera` com o `fx`/`fy` do `camera_intrinsics.yaml`; `commit.branch` igual à branch atual; `linhas_descartadas: 0` |
| 6 | Eventos | `cat $U/events.csv` | na ordem: `gravador,inicio` → `inspecao,SEARCHING` → `scan,goal_ativo` → `inspecao,CENTERING` → `center,goal_ativo` → `scan,goal_fim ... cancelado` → `center,goal_fim ... sucesso` → `inspecao,IDLE` → `gravador,fim`, com o fim ~1 s (`tail_s`) depois do IDLE |
| 7 | Pré-gravação | `head -3 $U/joints.csv` | primeiras linhas com `t` negativo (até −1 s, `pre_s`) |
| 8 | Repetição | start CENTER de novo | pasta `002_...`; status do repouso com `repetition: 3` |
| 9 | TRACK com perda | start TRACK; em TRACKING, cubra a garrafa ~2 s e descubra; depois `stop` | pasta `..._garrafa_track`; `desfecho: rastreou`, `readquisicoes: 1`; dois goals em `center`: o primeiro `abortado` com `alvo perdido`, o segundo `cancelado` |
| 10 | Sem alvo | tire a garrafa de cena; start CENTER | pasta `..._garrafa_nd`; `desfecho: sem_alvo`, `mensagem_final: Alvo não encontrado (varredura completa)`; sem `servo.csv` nem `center_feedback.csv`; `scan[0]` com `completed: true` |
| 11 | Goal direto | gerenciador em IDLE, `set_target` e `send_goal` CENTER | status `Gravando goal Center direto: ...`; `resumo.json` com `origem: direto`, `readquisicoes: null`, `desfecho: centralizado` (ou `falhou`, se a garrafa estiver fora da imagem: `mensagem_final: alvo perdido`) |
| 12 | Configuração recusada | `configure` durante uma inspeção | `success: false`, `Gravação em andamento (teste_gravador #NNN): configure depois do fim` |
| 13 | Modo de imagem | `configure` com `image_mode: 'foto'` | `success: false`, `image_mode="foto" desconhecido (opções: none, snapshots, video)` |
| 14 | Operador assume | durante a inspeção, um jog | IDLE `Inspeção abortada pelo operador`; a pasta fecha ~1 s depois; `events.csv` com `mux,fonte,web` |
| 15 | Ctrl+C no gravador | durante uma inspeção, Ctrl+C no terminal 3 | log `Gravado: ... (interrompido, X s)`; `resumo.json` com `desfecho: interrompido` e `motivo_interrupcao: nó encerrado`; a inspeção continua normalmente |
| 16 | Parâmetro inválido | `ros2 run pantilt_experiment experiment_recorder --ros-args -p image_mode:=foto` | `[FATAL] Parâmetro inválido: image_mode="foto" desconhecido (opções: none, snapshots, video)` e o nó sai |
| 17 | Pouco disco | terminal 3 com `-p min_free_gb:=100000.0`; start CENTER | log `Gravação não iniciada: pouco espaço em disco: X GB livres (mínimo 100000.0 GB)`; a inspeção segue sem gravar |
| 18 | Gravador sobe no meio | terminal 3 parado; start TRACK; em TRACKING, suba o terminal 3 | a gravação abre na hora (o `/inspection/status` é transient_local) e fecha no IDLE |
| 19 | Leitura no Python | `python3 -c "import numpy as np; d = np.genfromtxt('$U/servo.csv', delimiter=',', names=True, dtype=None, encoding='utf-8'); print(d['t'][:3], d['error_px'][:3])"` | três tempos crescentes e os erros em px |

**Preste atenção:**

- Durante a gravação a pasta se chama `NNN_AAAAMMDD_HHMMSS`; o equipamento e o modo entram no nome só ao fechar.
- **Rótulo:** vale para todas as gravações seguintes, até outro `configure`. Reiniciar o nó volta ao `default_label` (`sem_rotulo`); a numeração continua da maior pasta existente no rótulo.
- **Modo `nd`:** nenhum goal `Center` chegou a rodar (ex.: alvo não encontrado) ou o servo está com `publish_debug: false`.
- **`ausente` em `parametros`:** o nó não estava no ar (ou tem outro nome) quando a gravação abriu.
- **Tempo:** `t` é o instante da captura (o `header.stamp`) e `t_rx` o de chegada ao gravador. Para alvo, servo e detector, use `t`; para `cmd_*`, `center_feedback` e `scan_feedback` (sem header), `t = t_rx`. O `JointState` é carimbado pelo bridge na chegada da serial.
- O `commit` é lido quando o nó sobe: depois de mudar de branch ou de commitar, reinicie o gravador. `alteracoes_locais: true` indica arquivos versionados modificados sem commit.
- Tamanho típico: ~100 KB por 15 s de inspeção.
- `/ros2_ws/ensaios` fica fora do git e aparece no Windows na pasta do volume.

### 7.3 Imagens (etapa 3)

**O que valida:** os modos de imagem do `experiment_recorder` (architecture.md §4.11, "Imagens"):
- `snapshots`: arquivos `NN_<evento>.jpg` em cada entrada em CENTERING (ou no início do goal direto), no primeiro `centered=true` e no fim, depois da cauda;
- `video`: `video.mp4` com o `video_frames.csv`;
- em ambos, o custo no detector, medido pelo `timing.csv`.

**Pré-requisitos:** seção 7.2 aprovada. O teste 1 dispensa hardware e câmera; os demais precisam do sistema de inspeção funcionando.

**Build e testes automáticos (terminal 1):**

```bash
cd /ros2_ws && colcon build --symlink-install \
  --packages-select pantilt_interfaces pantilt_dataset pantilt_experiment pantilt_bringup && ws
cd /ros2_ws/src/pantilt_ros/pantilt_experiment && python3 -m pytest -q test && cd /ros2_ws
```

**Terminais 2 a 4:** os mesmos da seção 7.2. **Feche a página web** nos testes 2 e 6: com ela aberta, o detector já desenha a imagem de debug, e a comparação de custo perde o sentido.

**Terminal 5: comandos.** Troque `garrafa` por uma chave do `equipment_file` em uso.

```bash
ros2 service call /experiment/configure pantilt_interfaces/srv/ConfigureExperiment \
  "{label: 'teste_imagens', notes: 'roteiro 7.3', image_mode: 'snapshots'}"
ros2 service call /inspection/start pantilt_interfaces/srv/StartInspection "{equipment: 'garrafa', mode: 0}"
# trocar para vídeo
ros2 service call /experiment/configure pantilt_interfaces/srv/ConfigureExperiment \
  "{label: 'teste_imagens', notes: 'roteiro 7.3', image_mode: 'video'}"
# última pasta gravada
U=$(ls -td /ros2_ws/ensaios/teste_imagens/*/ | head -1); echo $U; ls $U
python3 -c "import json; print(json.load(open('$U/resumo.json'))['imagens'])"
grep snapshot $U/events.csv
# quadros do MP4 (deve bater com as linhas do video_frames.csv)
python3 -c "import cv2; c = cv2.VideoCapture('$U/video.mp4'); print(int(c.get(cv2.CAP_PROP_FRAME_COUNT)))"
wc -l $U/video_frames.csv
# taxa e inferência do detector numa gravação (média de inference_ms e quadros/s)
python3 -c "import numpy as np; d = np.genfromtxt('$U/timing.csv', delimiter=',', names=True); print(f'{1/np.diff(d[\"t\"]).mean():.1f} q/s, inferência {d[\"inference_ms\"].mean():.0f} ms')"
```

| # | Teste | Comando / ação | Esperado |
|---|---|---|---|
| 1 | pytest | terminal 1 | todos passam (`test_session.py`, `test_csv_writer.py` e `test_snapshot_writer.py`) |
| 2 | Sem imagem | `configure` com `image_mode: 'none'`; start CENTER; durante a inspeção, `ros2 topic info /perception/debug_image` | `Subscription count: 0` (com a página fechada); pasta sem `.jpg` nem `video.mp4`; `imagens: {'snapshots': [], 'video': None}` |
| 3 | Snapshots, CENTER | `configure` com `snapshots`; start CENTER | pasta com `01_inicio.jpg`, `02_centrado.jpg` e `03_fim.jpg`; o `inicio` mostra a bbox fora do centro e o `fim` a garrafa centralizada; `grep snapshot` com as três linhas, na ordem |
| 4 | Snapshots, readquisição | start TRACK; em TRACKING, cubra a garrafa ~2 s e descubra; depois `stop` | `01_inicio`, `02_centrado`, `03_inicio` (a readquisição) e `04_fim`; não há um segundo `centrado` (só o primeiro conta) |
| 5 | Snapshots, goal direto | gerenciador em IDLE; `set_target` e `send_goal` CENTER (comandos da §7.2) | `01_inicio`, `02_centrado` e `03_fim`. O `t` do `inicio` no `resumo.json` pode ser algumas décimas de segundo maior que zero: é o primeiro quadro desenhado depois que o detector descobre a assinatura |
| 6 | Vídeo | `configure` com `video`; start CENTER | `video.mp4` que abre no Windows e mostra as caixas; `CAP_PROP_FRAME_COUNT` igual às linhas do `video_frames.csv` menos o cabeçalho e a `imagens.video.quadros`; `descartados: 0`, `erro: ''` |
| 7 | Custo no detector | taxa e inferência do `timing.csv` nas gravações dos testes 2, 3 e 6 | anote os três valores; `snapshots` e `video` devem ficar próximos de `none` (a imagem de debug é desenhada, mas a inferência domina). Uma queda grande da taxa indica que o modo pesa no ensaio |
| 8 | Ctrl+C com vídeo | gravação `video` em andamento; Ctrl+C no terminal 3 | `desfecho: interrompido`; o `video.mp4` abre normalmente (finalizado) |
| 9 | Configuração | `configure` com `image_mode: 'foto'` | `success: false`, `image_mode="foto" desconhecido (opções: none, snapshots, video)` |

**Preste atenção:**

- Os snapshots são **a imagem de debug** (com as caixas e o alvo desenhados), não a imagem crua da câmera.
- O `fim` é tirado ao fechar, depois da cauda (`tail_s`): mostra o assentamento. Sem nenhum quadro recebido, não há `fim`.
- No vídeo, o `t` do `video_frames.csv` é o `header.stamp` do quadro: use-o para alinhar o vídeo com o `servo.csv`. A taxa declarada no MP4 (`video_fps`) é só nominal.
- Tamanho do vídeo: anote o do teste 6 (`du -h $U/video.mp4`) para dimensionar uma série de ensaios. O disco é conferido a cada 5 s (`min_free_gb`).

### 7.4 Bloco Ensaio na página (etapa 4)

**O que valida:** o bloco **Ensaio** da aba Inspeção (architecture.md §4.8):
- a linha de configuração (rótulo, observação, modo de imagem e **Aplicar**), que chama o `/experiment/configure`;
- a caixa **Gravação** e o chip `● ENSAIO` sobre o vídeo, que mostram o `/experiment/status`.

**Pré-requisitos:** seção 7.2 aprovada (e a 7.3, para os modos de imagem).

**Build (terminal 1):** a página é copiada para `share` no build.

```bash
cd /ros2_ws && colcon build --symlink-install --packages-select pantilt_web && ws
```

**Terminais:**
- **2:** `ros2 launch pantilt_bringup system.launch.py recorder:=false`;
- **3:** `ros2 run pantilt_experiment experiment_recorder --ros-args --params-file /ros2_ws/src/pantilt_ros/pantilt_bringup/config/params.yaml`;
- **4:** `ros2 topic echo /experiment/status`.

Abra `http://localhost:8080` e recarregue sem cache (Ctrl+F5), porque o navegador guarda os `.js` antigos.

| # | Teste | Como | Esperado |
|---|---|---|---|
| 1 | Layout | página em 1366×768 (ou a tela do notebook do ensaio) | a linha `ENSAIO` cabe entre o formulário e as caixas, e as quatro caixas (Estado, Equipamento, Erro, Gravação) e a linha de mensagem aparecem inteiras, sem corte. Se algo cortar, ajuste `--bottom-h` no `style.css` |
| 2 | Abertura | página com o gravador no ar | os campos mostram o rótulo do nó (`sem_rotulo`), a observação vazia e `Sem imagem`; a caixa Gravação mostra `próx. #NNN` |
| 3 | Gravador fora do ar | terminal 3 parado; recarregue a página | campos e Aplicar desabilitados; o rótulo mostra `Aguardando o experiment_recorder…` como placeholder; a caixa mostra `—` |
| 4 | Gravador sobe | suba o terminal 3 | em poucos segundos o bloco fica disponível, sem recarregar |
| 5 | Aplicar | rótulo `Teste Página`, observação `roteiro 7.4`, modo `Snapshots`; **Aplicar** | o Aplicar fica roxo cheio ao editar e volta ao contorno ao aplicar; toast verde `Próxima gravação: teste_pagina #001 \| imagens: snapshots \| obs.: roteiro 7.4`; o campo passa a `teste_pagina` (rótulo limpo pelo nó); terminal 4 com `label: teste_pagina` |
| 6 | Configuração pela CLI | `ros2 service call /experiment/configure pantilt_interfaces/srv/ConfigureExperiment "{label: 'cli', notes: '', image_mode: 'none'}"` | em ~0,5 s os campos mostram `cli`, a observação vazia e `Sem imagem` |
| 7 | Edição preservada | digite um rótulo sem aplicar e repita a chamada da CLI | o campo editado **não** é sobrescrito; o Aplicar continua destacado |
| 8 | Iniciar aplica antes | rótulo `teste_pagina` (sem Aplicar); **Iniciar** CENTER | a inspeção começa; a gravação sai em `/ros2_ws/ensaios/teste_pagina/`; o Aplicar volta ao contorno |
| 9 | Durante a gravação | observe durante a inspeção | caixa Gravação `● #001 00:05` em vermelho, com o tempo subindo; chip `● ENSAIO teste_pagina #001 00:05` no canto inferior direito do vídeo; campos e Aplicar desabilitados (também na cauda) |
| 10 | Fim da gravação | inspeção volta ao IDLE | ~1 s depois (cauda), toast `Gravado: teste_pagina/001_..._center (centralizado, X s)` por ~6 s; o chip some; a caixa mostra `próx. #002`; o tooltip da caixa mostra a mensagem e a pasta |
| 11 | Iniciar com recusa | com `tail_s` maior para dar tempo (terminal 3 com `-p tail_s:=10.0`): termine uma inspeção, edite o rótulo e clique em Iniciar durante a cauda | toast vermelho `Inspeção não iniciada: ensaio não configurado (Gravação em andamento ...)`; a inspeção não começa |
| 12 | Gravação não iniciada | terminal 3 com `-p min_free_gb:=100000.0`; Iniciar | a inspeção roda; toast `Gravação não iniciada: pouco espaço em disco ...`; nenhum chip |
| 13 | Recarregar durante a gravação | F5 no meio de uma inspeção gravada | o bloco volta direto em `● #NNN` com o tempo correto (status transient_local) |
| 14 | Gravador cai durante a gravação | Ctrl+C no terminal 3 no meio de uma inspeção | em ~2 s o bloco fica indisponível e o chip some; a inspeção continua |
| 15 | Goal direto | `send_goal` CENTER pela CLI (§7.2) com a página aberta | a caixa e o chip mostram a gravação do goal direto como numa inspeção |

**Preste atenção:**

- O bloco só configura as **próximas** gravações: o nó recusa mudanças durante uma gravação, inclusive na cauda.
- O rótulo vazio vale o `default_label` do nó (`sem_rotulo`).
- O Iniciar só aplica a configuração se ela foi editada; sem edição, nada é chamado antes do `/inspection/start`.

### 7.5 Análise dos ensaios (etapa 5)

**O que valida:** o `analisar_ensaios` (architecture.md §4.11 "Análise" e §10). Ele lê as pastas do `experiment_recorder` e gera em `<pasta>/_analise/`:
- `metricas.csv`, uma linha por gravação;
- `resumo_por_rotulo.csv`;
- as figuras em PNG e PDF.

**Pré-requisitos:** gravações reais de pelo menos dois rótulos, com algumas repetições cada (seção 7.2), e de preferência um TRACK, para o `e_d`. Os testes 1 e 2 dispensam hardware.

**Build e testes automáticos (terminal 1):**

```bash
cd /ros2_ws && colcon build --symlink-install --packages-select pantilt_experiment && ws
cd /ros2_ws/src/pantilt_ros/pantilt_experiment && python3 -m pytest -q test && cd /ros2_ws
```

**Comandos:**

```bash
ros2 run pantilt_experiment analisar_ensaios                       # /ros2_ws/ensaios
ros2 run pantilt_experiment analisar_ensaios --rotulos degrau_kp0.5 degrau_kp0.8
ros2 run pantilt_experiment analisar_ensaios --sem-figuras         # só os CSVs (rápido)
A=/ros2_ws/ensaios/_analise; ls $A $A/figuras
python3 -c "import csv; [print(r['rotulo'], r['repeticao'], r['desfecho'], r['t_c_s'], r['e_r_px'], r['mp_pan_deg'], r['iae_px_s']) for r in csv.DictReader(open('$A/metricas.csv'))]"
```

| # | Teste | Comando / ação | Esperado |
|---|---|---|---|
| 1 | pytest | terminal 1 | todos passam (inclui `test_analysis.py` e `test_plots.py`) |
| 2 | Sem gravações | `analisar_ensaios /tmp` | `Nenhuma gravação encontrada em /tmp` e código de saída 1 |
| 3 | Execução completa | `analisar_ensaios` | `N gravações analisadas -> /ros2_ws/ensaios/_analise`; resumo por rótulo no terminal (sucesso, `t_c`, `e_r`, IAE, `M_p` e `e_d` com média ± desvio); nenhum aviso de erro |
| 4 | `t_c` e `e_r` | compare o `metricas.csv` com o `resumo.json` de uma gravação CENTER | `t_c_s` e `e_r_px` iguais ao `convergence_time_s` e ao `final_error_px` do goal que convergiu |
| 5 | Readquisição | gravação TRACK com perda (§7.2 teste 9) | uma linha, com as métricas do goal que convergiu; `readquisicoes: 1`; `e_d_px` preenchido |
| 6 | Sem alvo e interrompida | gravações `sem_alvo` e `interrompido` da §7.2 | `sem_alvo`: métricas vazias e sem figura própria; `interrompido`: fica no `metricas.csv`, mas sai das estatísticas do `resumo_por_rotulo.csv` (linha `interrompidas` com a contagem) |
| 7 | Gravação em andamento | rode durante uma gravação | `Aviso: ... sem resumo.json (gravação em andamento?): ignorada` |
| 8 | Figura de uma gravação | abra um PNG de `figuras/<rótulo>/` | título com rótulo, repetição, modo, desfecho, `t_c`, `e_r`, IAE e `M_p`; quatro painéis (erro, θ, comando, juntas); linhas verticais de início do goal, primeira detecção, início da permanência e centralizado; vírgula decimal |
| 9 | Coerência da figura | na mesma figura | o início da permanência fica `t_c` depois da primeira detecção; o erro fica dentro da tolerância dali até o centralizado (`hold_time_s` depois) |
| 10 | Comparativas | `figuras/comparativo_metricas` e `comparativo_respostas` | um boxplot por rótulo, com os pontos de cada repetição; barras de sucesso com o `n`; respostas sobrepostas por rótulo (TRACK tracejado) |
| 11 | PDF | abra um `.pdf` no Windows | vetorial (o zoom não pixela), com texto selecionável |

**Preste atenção:**

- **Goal principal:** com readquisições, `t_c`, `e_r`, `M_p`, IAE e `e_d` são do goal da **primeira convergência** (o mesmo critério do `Center.Result`). Sem convergência, `t_c` e `e_r` ficam vazios, e não 0.
- **`M_p`:** é o maior `|θ|` do lado oposto ao do erro inicial, até o primeiro centralizado. O `%` fica vazio quando o eixo começou com menos de 1° de erro.
- **Primeira detecção antes do início do goal:** é normal. A primeira detecção usa o `header.stamp` (instante da captura), que precede a chegada do alvo ao servo por ~115 ms.
- **`--rotulos`:** os CSVs de `_analise` são reescritos só com os rótulos pedidos. Para a tabela final, rode sem filtro.
- **Figuras antigas:** figuras de gravações apagadas continuam em `_analise/figuras`. Apague a pasta `_analise` para regenerar do zero.
- **Tempo:** cada figura leva ~2 s (PNG mais PDF). Para conferir só os números, use `--sem-figuras`.
- **Decimal:** os CSVs usam ponto, que o numpy e as planilhas leem sem configuração. No Excel em português, importe com o separador decimal "." (Dados > De Texto/CSV).

---

## 8. Solução de problemas

| Sintoma | Causa provável | O que fazer |
|---|---|---|
| Build falha com `canonicalize_version() ... strip_trailing_zero` | setuptools voltou ao 84 | `pip install setuptools==58.2.0` |
| `ros2: command not found` ou pacote não encontrado | terminal sem o ambiente | `ws`; depois de um `colcon build`, `ws` de novo |
| `Falha ao abrir /dev/ttyUSB0` repetido | ESP32 não anexada ou em outra porta | `ls /dev/ttyUSB* /dev/ttyACM*`; refaça o `usbipd attach`; ajuste `device` no yaml |
| `Comando de posição rejeitado: ainda sem telemetria` | bridge ainda não recebeu telemetria | confira `ros2 topic hz /joint_states` e o `/ptu/errors` |
| `Código de erro desconhecido (N)` em `/ptu/errors` | código do firmware ainda não mapeado no host | consulte `docs/SerialProtocol.h` e atualize `ERROR_CODES` em `serial_protocol.py` |
| `offering incompatible QoS ... RELIABILITY_QOS_POLICY` no log do web_video_server | stream aberto sem `qos_profile=sensor_data` | use a URL da página ou inclua o parâmetro. As assinaturas RELIABLE antigas continuam listadas até reiniciar o `web.launch.py` |
| `Não foi possível abrir a fonte "0"` repetido | câmera não anexada ao WSL (o `/dev/video0` pode existir mesmo assim) | `ls /sys/class/video4linux/`; se estiver vazio, refaça o `usbipd attach` |
| `ros2 topic hz` em ~7 Hz ou vídeo em ~5 fps | processo sem o perfil do Fast DDS ou `/dev/shm` cheio | `echo $FASTRTPS_DEFAULT_PROFILES_FILE` no terminal do nó; `df -h /dev/shm`; `ros2 daemon stop` |
| Página não carrega no navegador do Windows (mas `curl localhost:8080` no container responde) | porta não publicada pelo docker-compose (só 8080, 9090 e 8081) | use as portas padrão do `web.launch.py`; o argumento é `http_port`/`video_port` (um `port:=` é ignorado sem aviso) |
| `localhost:8080` mostra uma lista de tópicos em vez da página | o web_video_server está na 8080 (web subida com `video_port:=8080`) | suba a web com as portas padrão (página 8080, vídeo 8081) |
| Página em `Servidor de vídeo indisponível` | web_video_server fora do ar ou em outra porta | confira o terminal da web; com outra porta, abra a página com `?video=<porta>` |
| Tópico não encontrado no stream | tópico com `%2F` na URL | use `/` literal: o web_video_server não decodifica `%2F` |
| Aba Coleta em `capture_node indisponível` | `capture_node` fora do ar ou travado (sem `/capture/status` há 2 s) | confira o terminal do launch; `ros2 node list`; suba de novo com o `dataset.launch.py` |
| `[FATAL] ... model_path "..." não encontrado` no detector | pesos não baixados ou em outra pasta | baixe com o comando da seção 2.2 ou ajuste `model_path` |
| Detector com poucos quadros/s ou atraso alto | inferência em CPU | reduza `imgsz` (480 ou 320); confira `free -h`; feche abas extras da página |
| `/perception/target` não publica nada | sem filtro ativo (é o esperado) | `ros2 service call /perception/set_target ...` com uma chave do yaml |
| Aba Inspeção em `Gerenciador indisponível (inspection_manager não respondeu)` | `inspection_manager` fora do ar ou iniciado em outro `ROS_DOMAIN_ID` | confira o terminal dele e `ros2 node list`; suba de novo com o comando da seção 6.1 |
| `/inspection/start` recusado com `Indisponível: ...` | servidor de action ou detector fora do ar (ou ainda subindo) | suba o launch indicado na mensagem e espere alguns segundos |
| `ros2 param dump` com valores diferentes do yaml | nó iniciado sem `--ros-args --params-file` (ex.: `--ros_param`, que é ignorado) | suba de novo com o comando da seção 2 |

---

## 9. Roteiros futuros

Seções a acrescentar aqui quando os itens existirem:

- `pantilt_control`: sintonia e ensaios do `visual_servo_node` (tarefa E) e `controllers/fuzzy.py`;
- `pantilt_web`: inspeção pela página (aba Inspeção com o `inspection_manager`);
