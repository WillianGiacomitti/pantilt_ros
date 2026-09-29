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
- [3. pantilt_web](#3-pantilt_web)
- [4. pantilt_control](#4-pantilt_control)
- [5. pantilt_dataset](#5-pantilt_dataset)
- [6. Solução de problemas](#6-solução-de-problemas)
- [7. Roteiros futuros](#7-roteiros-futuros)

| Pacote | O que valida | ESP32 | Câmera |
|---|---|---|---|
| `pantilt_hardware` | serial, telemetria, limites de ângulo, fail-safe, prioridade e watchdog do mux | sim (o mux pode ser testado sem ela) | não |
| `pantilt_perception` | `camera_node`: taxa, QoS, perda e retorno da câmera | não | sim |
| `pantilt_web` | vídeo na página, reconexão, jog com o hardware | só no jog | sim |
| `pantilt_control` | `scan_node`: varredura, cancelamento, prioridade do operador, abortos | sim | não |
| `pantilt_dataset` | `capture_node`: MP4 + `.json`, recusas, varredura durante a gravação, disco | só com varredura | sim |

Ordem recomendada na primeira vez: 1.1 → 1.2 → 1.3 → 1.4 → 4 → 5. A seção 1.2 é pré-requisito de qualquer teste em malha fechada (seções 4 e 5).

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
ps aux | grep -v grep | grep -E 'serial_bridge|command_mux|camera_node|scan_node|web_video_server'

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

**O que valida:** a comunicação com a ESP32 (`serial_bridge_node`), a telemetria dos encoders, os limites de ângulo e o fail-safe, e a arbitração de comandos (`command_mux`).

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

Confirma que a posição em `/joint_states` (lida dos encoders AS5600) acompanha o movimento real. **Não use `/control/scan` nem nada em malha fechada antes de estes testes passarem nos dois eixos.** Um encoder ruim produz um ângulo falso e plausível, e os limites do bridge e o `scan_node` confiam nele (histórico em `docs/diagnostico_encoders.md`).

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

O `command_mux` é o único caminho de comandos até o bridge (architecture.md §4.6). Ele assina `/ptu/cmd_vel_auto`, `/ptu/cmd_vel_web` e `/ptu/cmd_pos_web`, repassa para `/ptu/cmd_vel` e `/ptu/cmd_pos` e publica a fonte ativa (`web`, `auto` ou `none`) em `/ptu/control_source`.

- **Prioridade:** um comando da web é repassado na hora. Os comandos automáticos são descartados enquanto a web tiver publicado nos últimos `operator_hold_s` (1,0 s).
- **Watchdog:** depois de uma velocidade **não nula**, se a fonte ativa ficar em silêncio por mais de `cmd_timeout_s` (0,3 s), o mux envia velocidade zero uma única vez. Comandos de posição não armam o watchdog, porque um zero interromperia o movimento de posição.

Estes testes não precisam da ESP32. Sem ela, o `serial_bridge_node` fica registrando `Falha ao abrir /dev/ttyUSB0` a cada tentativa de reconexão; isso é esperado e não afeta o mux. Com o hardware, observe o eixo.

**Terminal 1:** `ros2 launch pantilt_bringup hardware.launch.py` (o mesmo da seção 1.1).

**Terminal 2: observação**

```bash
ros2 node info /command_mux                 # assina os 3 tópicos de entrada; publica /ptu/cmd_vel, /ptu/cmd_pos, /ptu/control_source
ros2 topic echo /ptu/control_source         # transient_local: mostra a fonte atual ao conectar
ros2 topic echo /ptu/cmd_vel --field angular
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

Para registrar: `ros2 bag record /ptu/cmd_vel_auto /ptu/cmd_vel_web /ptu/cmd_pos_web /ptu/cmd_vel /ptu/cmd_pos /ptu/control_source /joint_states`.

### Preste atenção

- **Nenhum `ERRO 4: Fail-safe acionado`** pode aparecer em `/ptu/errors` com o nó parado. Se aparecer, o heartbeat não está a 5 Hz.
- Os erros 8 a 18 do firmware (ímã do encoder, passos perdidos, driver, temperatura, limite do firmware, home) aparecem em `/ptu/errors` com a descrição. Os de encoder, passos e driver se repetem a cada 2 s enquanto a condição durar. `Código de erro desconhecido (N)` indica que o firmware tem um código novo que o `serial_protocol.py` ainda não conhece.
- Perto de ±29° podem surgir os erros 16/17 (limite de software **do firmware**). Anote qual limite age primeiro, o do bridge ou o do firmware.
- Se no teste 3 da seção 1.3 o pan passar de 29° de forma relevante, aumente `limit_margin_deg` no `params.yaml`.
- Limitação conhecida: se o mux for morto com `kill -9` com o eixo em movimento, nada envia o zero, e o heartbeat do bridge impede o fail-safe do firmware. Só os limites de ângulo param o eixo nesse caso.

---

## 2. pantilt_perception

**O que valida:** o `camera_node` publicando `/camera/image_raw` com a taxa e o QoS certos, e o comportamento quando a câmera some e volta.

**Terminal 1: câmera.** Até existir o `perception.launch.py`, o nó sobe com `ros2 run`:

```bash
ros2 run pantilt_perception camera_node --ros-args \
  --params-file /ros2_ws/src/pantilt_ros/pantilt_bringup/config/params.yaml
```

Esperado no log:

```
Fonte: device "0" | pedido 640x480 @ 30.0 fps | frame_id "camera_optical_frame"
Fonte aberta: 640x480 @ 30.0 fps (MJPG)
```

Se aparecer `A fonte entrega WxH em vez de 640x480`, a câmera não aceitou a resolução pedida. O nó publica no tamanho nativo.

**Terminal 2: verificação**

```bash
ros2 param dump /camera_node                # confere que o params.yaml foi lido
ros2 topic hz /camera/image_raw             # ~19-30 Hz conforme a luz; ~7 Hz indica problema de transporte (seção 6)
ros2 topic info -v /camera/image_raw        # publisher camera_node: Reliability BEST_EFFORT
ros2 topic echo /camera/image_raw --field header   # stamp avançando e frame_id camera_optical_frame
```

Anote a taxa desta etapa: ela é a referência para a seção 3.

| # | Teste | Como | Esperado |
|---|---|---|---|
| 1 | Parar a câmera | Ctrl+C no terminal 1 | com a página aberta, o vídeo congela no último quadro (anote se a página muda para `Aguardando imagens`) |
| 2 | Voltar a câmera | suba o `camera_node` de novo | anote se o vídeo da página volta sozinho ou só ao recarregar |
| 3 | Câmera desconectada | `usbipd detach --busid <BUSID>` no Windows | log `Fonte "0" parou de entregar quadros; reabrindo` e depois `Não foi possível abrir a fonte "0"` a cada 5 s |
| 4 | Câmera reconectada | `usbipd attach --wsl --busid <BUSID>` | anote se o nó volta a abrir a fonte sozinho (`Fonte aberta: ...`) ou se precisa ser reiniciado |

Para registrar (opcional): `ros2 bag record /camera/image_raw`. Ocupa ~20 MB/s a 640×480 e 20 Hz; o `/ros2_ws` fica no `C:` do Windows, com pouco espaço. Grave poucos segundos.

### Preste atenção

- O nó precisa ser iniciado com `--ros-args --params-file`. Um `ros2 param dump` com valores diferentes do yaml indica que o arquivo não foi lido.
- Taxa em ~7 Hz é problema de transporte (perfil do Fast DDS ou `/dev/shm`), não da câmera.
- Com pouca luz, a exposição automática derruba a taxa. Compare as medidas sempre com a mesma iluminação.

---

## 3. pantilt_web

**O que valida:** o caminho `camera_node` → `/camera/image_raw` → `web_video_server` → página, a reconexão da página ao rosbridge e o jog com o hardware (a imagem se move junto com o pan-tilt).

Enquanto o `detector_node` não existe, a página mostra `/camera/image_raw` no lugar do `/perception/debug_image`, via `?video_topic=`.

Endereços (no navegador do Windows):

| O quê | URL |
|---|---|
| Página | `http://localhost:8000/?video_topic=/camera/image_raw` |
| Stream direto | `http://localhost:8080/stream?topic=/camera/image_raw&qos_profile=sensor_data` |
| Tópicos que o web_video_server enxerga | `http://localhost:8080/` |

O `qos_profile=sensor_data` é obrigatório no stream direto. A página já o inclui (`web/js/config.js`).

**Terminais:**

1. câmera, como na seção 2;
2. observação (`ros2 topic hz`, `ros2 topic info -v`);
3. web: `ros2 launch pantilt_web web.launch.py`;
4. hardware (só para o jog): `ros2 launch pantilt_bringup hardware.launch.py`.

Esperado no log do terminal 3: `Waiting For connections on 0.0.0.0:8080` (web_video_server) e `Rosbridge WebSocket server started on port 9090`.

Abra a página: `http://localhost:8000/?video_topic=/camera/image_raw`.

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

- Se a assinatura do `web_video_server` aparecer como **RELIABLE**, o stream foi aberto sem `qos_profile=sensor_data` e não recebe quadros (seção 6).
- Anote o sentido da imagem em cada eixo nos testes 5 e 7. É isso que define o sinal do erro do `visual_servo_node` (`invert_pan`/`invert_tilt`).
- Com o sistema todo rodando, `df -h /dev/shm` deve mostrar algumas dezenas de MB usados, longe de 1 GB.

---

## 4. pantilt_control

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
- Pequenas correções no eixo que não faz parte do trecho (ex.: o tilt se mexendo enquanto o pan varre) são conhecidas: o nó corrige o ruído de leitura desse eixo. As melhorias estão listadas em `docs/diagnostico_encoders.md` §4 e ainda não foram aplicadas.
- Perto de cada alvo a velocidade cai (até 2°/s); a ultrapassagem deve ficar abaixo de ~1°. Anote se passar disso.
- A varredura usa ±28° e o bridge limita em ±29°: o log `Limite de ângulo atingido` não deve aparecer. Se aparecer, a ultrapassagem está grande demais.

---

## 5. pantilt_dataset

**O que valida:** o `capture_node` (architecture.md §4.9), que grava `/camera/image_raw` em MP4, com um `.json` de metadados ao lado, e opcionalmente mantém a varredura ativa com goals `Scan` em sequência. Os vídeos vão para `/ros2_ws/datasets/` (no Windows: `ros2_ws\datasets`), com o nome `AAAAMMDD_HHMMSS_<sessao>.mp4`.

**Pré-requisitos:** a câmera funcionando (seção 2). Para os testes com varredura, também o hardware e o `scan_node`, com a seção 1.2 aprovada (como na seção 4). Confira o espaço livre antes: `df -h /ros2_ws`.

Até existir o `dataset.launch.py` (Etapa 3), os nós sobem com `ros2 run`.

**Terminal 1: câmera** (o mesmo comando da seção 2)

```bash
ros2 run pantilt_perception camera_node --ros-args \
  --params-file /ros2_ws/src/pantilt_ros/pantilt_bringup/config/params.yaml
```

**Terminal 2: captura**

```bash
ros2 run pantilt_dataset capture_node --ros-args \
  --params-file /ros2_ws/src/pantilt_ros/pantilt_bringup/config/params.yaml
```

Esperado no log: `Saída: /ros2_ws/datasets | 15.0 fps | fourcc "mp4v" | mínimo 1.0 GB livres | fila 30`.

**Terminal 3: observação**

```bash
ros2 topic echo /capture/status             # transient_local: mostra o estado atual ao conectar; 2 Hz
ls -la /ros2_ws/datasets/
```

**Terminal 4: comandos**

```bash
# início sem varredura
ros2 service call /capture/start pantilt_interfaces/srv/StartCapture "{session: 'Mesa Janela (tarde)', scan: false, speed_deg_s: 0.0}"
# início com varredura (0 = velocidade do scan_node)
ros2 service call /capture/start pantilt_interfaces/srv/StartCapture "{session: 'mesa', scan: true, speed_deg_s: 0.0}"
# fim
ros2 service call /capture/stop std_srvs/srv/Trigger
```

**Testes sem hardware** (só a câmera)

| # | Teste | Comando / ação | Esperado |
|---|---|---|---|
| 1 | Gravação simples | início sem varredura, ~20 s, fim | `accepted=True` com `file` terminando em `_mesa_janela_tarde.mp4`; status com `recording: true`, `frames` subindo, `dropped: 0`, `message: Gravando`; no fim, `Gravado: N quadros em X s (F fps, M MB) -> <arquivo>`; MP4 e `.json` na pasta |
| 2 | Conteúdo do `.json` | `cat /ros2_ws/datasets/<nome>.json` | `frames`, `duration_s`, `fps_real` (~15), `width`/`height` 640×480, `size_bytes`, `scan.enabled: false`, `end_reason: Parada pelo operador` |
| 3 | Vídeo no Windows | abra o MP4 pelo Explorer | toca normalmente; anote o tamanho por minuto (`size_bytes` / `duration_s`) |
| 4 | Início duplicado | com uma gravação ativa, outro início | `accepted=False`, `Já existe uma gravação ativa (...)`; a gravação atual continua |
| 5 | Fim sem gravação | `/capture/stop` sem gravação ativa | `success=False`, `Nenhuma gravação ativa` |
| 6 | Câmera parou | durante a gravação, Ctrl+C no terminal 1; depois suba a câmera de novo | em ~2 s, status `Sem quadros de /camera/image_raw há mais de 2 s (a câmera está rodando?)` e um aviso no log; a gravação continua; com a câmera de volta, o status volta a `Gravando` |
| 7 | Gravação sem quadros | câmera parada; início e fim | `Nenhum quadro recebido; nada gravado`; nenhum arquivo criado |
| 8 | Varredura sem `scan_node` | início com `scan: true` sem o `control.launch.py` | `accepted=False`, `scan_node indisponível: suba o control.launch.py ou use scan=false` |
| 9 | Velocidade inválida | início com `speed_deg_s: 45.0` | `accepted=False`, `speed_deg_s=45.0 fora de [0, 30]°/s` |
| 10 | Ctrl+C no nó | durante a gravação, Ctrl+C no terminal 2 | log `Nó encerrado. Gravado: ...`; MP4 e `.json` completos, com `end_reason: Nó encerrado` |
| 11 | Pouco espaço | com o nó parado: `ros2 run pantilt_dataset capture_node --ros-args --params-file <params.yaml> -p min_free_gb:=10000.0`; depois um início | `accepted=False`, `Pouco espaço em disco: X GB livres (mínimo 10000.0 GB)` |
| 12 | Parâmetro inválido | `ros2 run pantilt_dataset capture_node --ros-args -p fourcc:=mp4` | `[FATAL] Parâmetro inválido: fourcc="mp4" inválido ...` e o nó sai |

**Testes com varredura** (terminais extras com `ros2 launch pantilt_bringup hardware.launch.py` e `ros2 launch pantilt_bringup control.launch.py`; mecanismo zerado no centro; STOP à mão)

| # | Teste | Comando / ação | Esperado |
|---|---|---|---|
| 13 | Gravação com varredura | início com `scan: true` | status `scanning: true`, `Gravando com varredura (passada 1)`; o `scan_node` registra `Varredura iniciada`; ao fim do padrão, `Varredura terminada: Padrão completo` e logo outra `Varredura iniciada` (passada 2), sem parar a gravação |
| 14 | Jog durante a gravação | segure um botão do jog na página | fonte `web`, o eixo obedece ao jog; a gravação não para; ao soltar, a varredura continua (como no teste 6 da seção 4) |
| 15 | Fim durante a varredura | `/capture/stop` com a varredura ativa | o `scan_node` registra `Varredura cancelada` e os eixos param; `.json` com `scan.goals_sent`/`goals_succeeded` |
| 16 | Varredura interrompida | durante a gravação com varredura, Ctrl+C no `control.launch.py` | status `scanning: false`, `Gravando sem varredura: interrompida, ...` (ex.: `scan_node saiu do ar`); a gravação continua; nenhum goal novo é enviado |
| 17 | Ctrl+C durante a varredura | suba o controle de novo, início com varredura e Ctrl+C no terminal 2 | o `scan_node` registra `Varredura cancelada`; arquivo finalizado com `end_reason: Nó encerrado` |

### Preste atenção

- **Disco:** o `/ros2_ws` fica no `C:` do Windows, com pouco espaço livre. O nó recusa o início e encerra a gravação (com o arquivo finalizado) abaixo de `min_free_gb`, verificando a cada 5 s. Anote no teste 3 quanto ocupa cada minuto, para planejar a coleta.
- **`dropped` > 0** significa que o disco ou a codificação não acompanham a câmera. Anote se aparecer; um `queue_size` maior só adia o problema.
- **Vídeo acelerado:** o MP4 declara `record_fps` (15). Se a câmera entregar menos (pouca luz), o vídeo toca mais rápido que a cena real. O `fps_real` do `.json` mostra a taxa gravada. Para extrair quadros para o dataset, isso não importa.
- Uma varredura interrompida **não é retomada** sozinha: para voltar a varrer, encerre e inicie de novo com `scan: true`.
- Se o `/capture/status` parar de chegar, o `capture_node` travou ou saiu. Confira o terminal 2.

---

## 6. Solução de problemas

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
| Página em `Servidor de vídeo indisponível` | web_video_server fora do ar ou em outra porta | confira o terminal da web; com outra porta, abra a página com `?video=<porta>` |
| Tópico não encontrado no stream | tópico com `%2F` na URL | use `/` literal: o web_video_server não decodifica `%2F` |
| `ros2 param dump` com valores diferentes do yaml | nó iniciado sem `--ros-args --params-file` (ex.: `--ros_param`, que é ignorado) | suba de novo com o comando da seção 2 |

---

## 7. Roteiros futuros

Seções a acrescentar aqui quando os itens existirem:

- `pantilt_dataset`: painel de coleta na página e `dataset.launch.py` (Etapa 3; entra na seção 5);
- `pantilt_perception`: `detector_node` (a página passa a abrir sem `?video_topic=`, mostrando `/perception/debug_image`);
- `pantilt_control`: `visual_servo_node` (PID e fuzzy);
- `pantilt_manager`: `inspection_manager` e inspeção pela página.
