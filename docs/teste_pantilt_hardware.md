# Roteiro de teste do serial_bridge_node com a ESP32 real

## Contexto

O `serial_bridge_node` já foi migrado para o pacote `pantilt_hardware` (branch `pantilt_hardware`) e testado apenas contra uma ESP32 simulada. Este roteiro serve para o primeiro teste com o hardware real, depois de reiniciar o container com a ESP32 anexada. Não há mudança de código; são só comandos para você executar.

Os dois nós do pacote (`serial_bridge_node` e `command_mux`) sobem juntos pelo `pantilt_bringup/launch/hardware.launch.py`, com os parâmetros do `pantilt_bringup/config/params.yaml`.

Conversão útil (os tópicos usam rad): 5° = 0.0873 · 10° = 0.1745 · 20° = 0.3491 · 29° = 0.5061 · 1 rad = 57.3°.

---

## 1. Antes de subir o container (Windows)

```powershell
usbipd list                              # anote o BUSID da ESP32
usbipd attach --wsl --busid <BUSID>
```

## 2. Preparação (terminal 1, dentro do container)

```bash
# branch certa (o volume reflete o que está checado no host)
cd /ros2_ws/src/pantilt_ros && git branch --show-current     # deve ser pantilt_hardware

# setuptools: a correção de hoje se perde se o container foi recriado
pip show setuptools | grep Version        # precisa ser 58.2.0
pip install setuptools==58.2.0            # só se não for

# porta serial visível e livre
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-id/ 2>/dev/null
ps aux | grep -v grep | grep -E 'serial_bridge|command_mux'   # nenhum nó antigo pode estar rodando (ex.: iniciado pelo entrypoint)

# build e testes
ws                                        # entra em /ros2_ws e carrega o ambiente
colcon build --symlink-install && ws
colcon test --packages-select pantilt_hardware && colcon test-result --verbose   # 32 testes, 0 falhas
```

Se o build falhar com `canonicalize_version() ... strip_trailing_zero`, o setuptools voltou ao 84: repita o `pip install` acima.

## 3. Rodar os nós (terminal 1)

```bash
ros2 launch pantilt_bringup hardware.launch.py
```

Para trocar a porta ou os limites, edite o `pantilt_bringup/config/params.yaml`. Com `--symlink-install`, a mudança vale sem rebuild: basta reiniciar o launch. Para testar sem mexer no arquivo do repositório, use uma cópia:

```bash
cp /ros2_ws/src/pantilt_ros/pantilt_bringup/config/params.yaml /tmp/params_teste.yaml
# edite em /tmp/params_teste.yaml, por exemplo: device: "/dev/ttyACM0" ou tilt_limits_deg: [-45.0, 45.0]
ros2 launch pantilt_bringup hardware.launch.py params_file:=/tmp/params_teste.yaml
```

Esperado no log:

- `[serial_bridge_node]: Limites efetivos: pan [-29.0°, 29.0°] ...`;
- `[serial_bridge_node]: Conectado a /dev/ttyUSB0 @ 921600 bps` e, se a ESP32 reiniciar ao abrir a porta, `[ESP32] Boot detectado ...`;
- `[command_mux]: Prioridade do operador: 1.00 s | watchdog de comando: 0.30 s`;
- `[command_mux]: Fonte de controle inicial: none`.

Ao encerrar com Ctrl+C, pode aparecer um `KeyboardInterrupt` no traceback do `serial_bridge_node`: o `ros2 launch` repassa um segundo SIGINT enquanto o nó já está encerrando. A velocidade zero é enviada antes desse ponto (`destroy_node`), então o eixo para mesmo assim.

## 4. Observação (terminal 2)

Em todo terminal novo:

```bash
docker exec -it ptu_web_bridge bash
ws
```

```bash
ros2 node list                              # /serial_bridge_node e /command_mux
ros2 node info /serial_bridge_node          # assina /ptu/cmd_vel e /ptu/cmd_pos; publica /joint_states e /ptu/errors; serviço /ptu/set_zero
ros2 topic list                             # /joint_states, /ptu/cmd_pos, /ptu/cmd_vel, /ptu/errors e os tópicos do mux (/ptu/cmd_vel_auto, /ptu/cmd_vel_web, /ptu/cmd_pos_web, /ptu/control_source)
ros2 param dump /serial_bridge_node         # confira heartbeat_hz: 5.0 e os limites

ros2 topic hz /joint_states                 # ~20 Hz (taxa da telemetria do firmware)
ros2 topic echo /joint_states --field position   # [pan, tilt] em rad
ros2 topic echo /ptu/errors                 # diagnósticos da ESP32 (transient_local: mostra também os antigos)
```

Deixe o `/ptu/errors` aberto num terminal durante todo o teste. **Nenhum `ERRO 4: Fail-safe acionado` pode aparecer com o nó parado**; esse era o bug do heartbeat de 2 Hz.

Para acompanhar os comandos que chegam ao bridge, use `ros2 topic echo /ptu/cmd_vel` e `ros2 topic echo /ptu/cmd_pos`.

## 5. Acionamento (terminal 3)

> ⚠️ **Nesta seção os comandos vão direto para o bridge (`/ptu/cmd_vel` e `/ptu/cmd_pos`). O `command_mux` está rodando, mas é contornado: não há watchdog de comando.** O firmware mantém a última velocidade recebida, e o heartbeat do bridge impede o fail-safe. Depois de qualquer comando de velocidade, **sempre envie o comando de parada**. Prefira `--once` a `-r`: interromper um `-r 10` com Ctrl+C **não** para o eixo. Deixe o comando de parada já digitado num terminal.

**Parada (use à vontade):**
```bash
ros2 topic pub --once /ptu/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.0, y: 0.0}}"
```

**Velocidade** (`angular.z` = pan, `angular.y` = tilt, em rad/s; comece devagar):
```bash
ros2 topic pub --once /ptu/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.1}}"    # pan +5.7°/s
ros2 topic pub --once /ptu/cmd_vel geometry_msgs/msg/Twist "{angular: {z: -0.1}}"   # pan -5.7°/s
ros2 topic pub --once /ptu/cmd_vel geometry_msgs/msg/Twist "{angular: {y: 0.1}}"    # tilt +5.7°/s
ros2 topic pub --once /ptu/cmd_vel geometry_msgs/msg/Twist "{angular: {y: -0.1}}"   # tilt -5.7°/s
```
No primeiro comando, anote para que lado o eixo gira fisicamente com o sinal positivo e se o `/joint_states` cresce. Isso define `invert_pan`/`invert_tilt` do `visual_servo_node` mais tarde.

**Posição** (absoluta, em rad):
```bash
ros2 topic pub --once /ptu/cmd_pos sensor_msgs/msg/JointState "{name: [pan_joint, tilt_joint], position: [0.1745, 0.0]}"  # pan 10°, tilt 0°
ros2 topic pub --once /ptu/cmd_pos sensor_msgs/msg/JointState "{name: [tilt_joint], position: [0.1745]}"               # só tilt 10°; pan fica onde está
ros2 topic pub --once /ptu/cmd_pos sensor_msgs/msg/JointState "{name: [pan_joint, tilt_joint], position: [0.0, 0.0]}"     # volta ao zero
```

**Zerar os eixos:**
```bash
ros2 service call /ptu/set_zero std_srvs/srv/Trigger     # esperado: success=True, message='Eixos zerados'
```
Os limites de software são **relativos ao zero**. Só zere com o mecanismo no centro mecânico real; caso contrário, os ±29° ficam deslocados.

## 6. Testes de segurança (nesta ordem)

| # | Teste | Comando | Esperado |
|---|---|---|---|
| 1 | Heartbeat | nó parado por ~1 min | nenhum `Fail-safe acionado` em `/ptu/errors` |
| 2 | Recorte de posição | `cmd_pos` com `position: [1.0, 0.0]` | pan vai a ~29° (0.506 rad); log `Comando de posição recortado ao limite` |
| 3 | Limite por velocidade | volte ao zero e envie `cmd_vel` `{angular: {z: 0.1}}` **sem** parar | pan para sozinho antes de 29°; log `Limite de ângulo atingido` |
| 4 | Bloqueio só no sentido do limite | com pan em ~29°: `z: 0.1` e depois `z: -0.1` | `+0.1` ignorado (log `Comando de velocidade limitado`); `-0.1` afasta do limite (depois pare) |
| 5 | Parada no Ctrl+C | eixo em movimento lento → Ctrl+C no terminal 1 (encerra o launch) | eixo para imediatamente (o bridge envia velocidade zero ao sair); o traceback descrito na seção 3 pode aparecer |
| 6 | Fail-safe do firmware | suba o launch de novo; eixo em movimento → `pkill -9 -f 'lib/pantilt_hardware/serial_bridge_node'` | eixo para em ~0,5 s (sem o zero do bridge; quem para é o fail-safe); o launch registra `process has died` e o `command_mux` continua rodando |
| 7 | Reconexão | desconecte o USB com o nó rodando e reconecte (e refaça o `usbipd attach`) | `/ptu/errors`: `Conexão serial ... perdida - reconectando...` e depois `Conectado a ...`; o caminho da porta precisa ser o mesmo |
| 8 | Parâmetro inválido | com o launch parado: `ros2 run pantilt_hardware serial_bridge_node --ros-args -p heartbeat_hz:=2.0` | `[FATAL] heartbeat_hz=2.0 inválido ...` e o nó sai |

O teste 8 usa `ros2 run` de propósito: o `-p` sobrescreve um único parâmetro, sem editar o `params.yaml`.

Para registrar o ensaio (opcional): `ros2 bag record /joint_states /ptu/cmd_vel /ptu/cmd_pos /ptu/errors`.

## 7. Teste do command_mux

O `command_mux` é o único caminho de comandos até o bridge (architecture.md §4.6). Ele assina `/ptu/cmd_vel_auto`, `/ptu/cmd_vel_web` e `/ptu/cmd_pos_web`, repassa para `/ptu/cmd_vel` e `/ptu/cmd_pos` e publica a fonte ativa (`web`, `auto` ou `none`) em `/ptu/control_source`.

- **Prioridade:** um comando da web é repassado na hora. Os comandos automáticos são descartados enquanto a web tiver publicado nos últimos `operator_hold_s` (1,0 s).
- **Watchdog:** depois de uma velocidade **não nula**, se a fonte ativa ficar em silêncio por mais de `cmd_timeout_s` (0,3 s), o mux envia velocidade zero uma única vez. Comandos de posição não armam o watchdog, porque um zero interromperia o movimento de posição.

Os testes 1 a 6 não precisam da ESP32. Sem ela, o `serial_bridge_node` do launch fica registrando `Falha ao abrir /dev/ttyUSB0` a cada tentativa de reconexão; isso é esperado e não afeta o mux. Com o hardware, observe o eixo.

**Terminal 1: launch**, o mesmo da seção 3:
```bash
ros2 launch pantilt_bringup hardware.launch.py
```
Esperado no log do mux: `Prioridade do operador: 1.00 s | watchdog de comando: 0.30 s` e `Fonte de controle inicial: none`.

**Terminal 2: observação**
```bash
ros2 node info /command_mux                 # assina os 3 tópicos de entrada; publica /ptu/cmd_vel, /ptu/cmd_pos, /ptu/control_source
ros2 topic echo /ptu/control_source         # transient_local: mostra a fonte atual ao conectar
ros2 topic echo /ptu/cmd_vel --field angular
```

**Terminal 3: acionamento**. Agora use os tópicos de entrada do mux, **nunca** `/ptu/cmd_vel` direto.

| # | Teste | Comando | Esperado |
|---|---|---|---|
| 1 | Repasse automático | `ros2 topic pub -r 10 /ptu/cmd_vel_auto geometry_msgs/msg/Twist "{angular: {z: 0.1}}"` | `/ptu/cmd_vel` recebe `z: 0.1` a 10 Hz; fonte `auto` |
| 2 | Watchdog auto | Ctrl+C no `pub` do teste 1 | em ~0,3 s um **único** `z: 0.0` em `/ptu/cmd_vel`; log `Watchdog: fonte "auto" em silêncio ...`; fonte `none`. Com hardware, o eixo para sozinho |
| 3 | Prioridade do operador | repita o `pub` do teste 1 e, em outro terminal: `ros2 topic pub --once /ptu/cmd_vel_web geometry_msgs/msg/Twist "{angular: {y: 0.1}}"` | fonte `web` na hora; `y: 0.1` repassado; o auto é descartado por 1 s (log `Comando automático descartado`); após 0,3 s sai o zero do watchdog (`fonte "web"`); passado 1 s, o auto volta (fonte `auto`) |
| 4 | Posição sem watchdog | `ros2 topic pub --once /ptu/cmd_pos_web sensor_msgs/msg/JointState "{name: [pan_joint, tilt_joint], position: [0.1745, 0.0]}"` | repassado em `/ptu/cmd_pos`; fonte `web` por 1 s e depois `none`; **nenhum** zero em `/ptu/cmd_vel` (com hardware, o eixo chega aos 10°) |
| 5 | Parada no Ctrl+C | com o `pub` do teste 1 rodando, Ctrl+C no terminal 1 (encerra o launch) | log `Encerrando: enviando velocidade zero` do mux; `/ptu/cmd_vel` recebe `z: 0.0`; o bridge também envia zero ao sair |
| 6 | Parâmetro inválido | com o launch parado: `ros2 run pantilt_hardware command_mux --ros-args -p cmd_timeout_s:=0.0` | `[FATAL] Parâmetro inválido: cmd_timeout_s deve ser positivo ...` e o nó sai |

Limitação: se o mux for morto com `kill -9` com o eixo em movimento, nada envia o zero, e o heartbeat do bridge impede o fail-safe do firmware. Só os limites de ângulo do bridge param o eixo nesse caso.

Para registrar: `ros2 bag record /ptu/cmd_vel_auto /ptu/cmd_vel_web /ptu/cmd_pos_web /ptu/cmd_vel /ptu/cmd_pos /ptu/control_source /joint_states`.

## Observações

- A página web atual (`web/index.html`) é legado e só serve de referência para o pacote `pantilt_web`. Não é preciso subir o rosbridge para estes testes.
- Se no teste 3 o pan passar de 29° de forma relevante, aumente `limit_margin_deg` no `params.yaml`. No simulador ele parou em 29,3°.
- Anote para o próximo passo (`command_mux`): o sentido dos eixos e se o tilt alcança ±90° com segurança.
