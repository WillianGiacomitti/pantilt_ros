# Roteiro de teste do serial_bridge_node com a ESP32 real

## Contexto

O `serial_bridge_node` já foi migrado para o pacote `pantilt_hardware` (branch `pantilt_hardware`) e testado apenas contra uma ESP32 simulada. Este roteiro serve para o primeiro teste com o hardware real, depois de reiniciar o container com a ESP32 anexada. Não há mudança de código; são só comandos para você executar.

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
ps aux | grep -v grep | grep serial_bridge   # nenhum bridge antigo pode estar rodando (ex.: iniciado pelo entrypoint)

# build e testes
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select pantilt_hardware && colcon test-result --verbose   # 15 testes, 0 falhas
```

Se o build falhar com `canonicalize_version() ... strip_trailing_zero`, o setuptools voltou ao 84: repita o `pip install` acima.

## 3. Rodar o nó (terminal 1)

```bash
ros2 run pantilt_hardware serial_bridge_node --ros-args \
  --params-file /ros2_ws/src/pantilt_ros/pantilt_bringup/config/params.yaml
```

Sobrescritas úteis (o `-p` vem **depois** do `--params-file` para ter prioridade):

```bash
  -p device:=/dev/ttyACM0                      # se a porta não for ttyUSB0
  -p tilt_limits_deg:="[-45.0, 45.0]"          # se o tilt da montagem não alcança ±90° com segurança
```

Esperado no log: `Limites efetivos: pan [-29.0°, 29.0°] ...`, `Conectado a /dev/ttyUSB0 @ 921600 bps` e, se a ESP32 reiniciar ao abrir a porta, `[ESP32] Boot detectado ...`.

## 4. Observação (terminal 2)

Em todo terminal novo:

```bash
docker exec -it <container> bash
source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash
```

```bash
ros2 node list                              # /serial_bridge_node
ros2 node info /serial_bridge_node          # assina /ptu/cmd_vel e /ptu/cmd_pos; publica /joint_states e /ptu/errors; serviço /ptu/set_zero
ros2 topic list                             # /joint_states, /ptu/cmd_pos, /ptu/cmd_vel, /ptu/errors
ros2 param dump /serial_bridge_node         # confira heartbeat_hz: 5.0 e os limites

ros2 topic hz /joint_states                 # ~20 Hz (taxa da telemetria do firmware)
ros2 topic echo /joint_states --field position   # [pan, tilt] em rad
ros2 topic echo /ptu/errors                 # diagnósticos da ESP32 (transient_local: mostra também os antigos)
```

Deixe o `/ptu/errors` aberto num terminal durante todo o teste. **Nenhum `ERRO 4: Fail-safe acionado` pode aparecer com o nó parado**; esse era o bug do heartbeat de 2 Hz.

Para acompanhar os comandos que chegam ao bridge, use `ros2 topic echo /ptu/cmd_vel` e `ros2 topic echo /ptu/cmd_pos`.

## 5. Acionamento (terminal 3)

> ⚠️ **Sem o `command_mux` ainda não há watchdog de comando.** O firmware mantém a última velocidade recebida, e o heartbeat do bridge impede o fail-safe. Depois de qualquer comando de velocidade, **sempre envie o comando de parada**. Prefira `--once` a `-r`: interromper um `-r 10` com Ctrl+C **não** para o eixo. Deixe o comando de parada já digitado num terminal.

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
| 5 | Parada no Ctrl+C | eixo em movimento lento → Ctrl+C no terminal 1 | eixo para imediatamente (o bridge envia velocidade zero ao sair) |
| 6 | Fail-safe do firmware | eixo em movimento → `pkill -9 -f 'lib/pantilt_hardware/serial_bridge_node'` | eixo para em ~0,5 s (sem o zero do bridge; quem para é o fail-safe) |
| 7 | Reconexão | desconecte o USB com o nó rodando e reconecte (e refaça o `usbipd attach`) | `/ptu/errors`: `Conexão serial ... perdida - reconectando...` e depois `Conectado a ...`; o caminho da porta precisa ser o mesmo |
| 8 | Parâmetro inválido | `ros2 run pantilt_hardware serial_bridge_node --ros-args -p heartbeat_hz:=2.0` | `[FATAL] heartbeat_hz=2.0 inválido ...` e o nó sai |

Para registrar o ensaio (opcional): `ros2 bag record /joint_states /ptu/cmd_vel /ptu/cmd_pos /ptu/errors`.

## Observações

- A página web atual **não controla** o PTU nesta branch, porque ela publica em `/ptu/cmd_*_web` e ainda não existe o `command_mux`. Não é preciso subir o rosbridge para este teste.
- Se no teste 3 o pan passar de 29° de forma relevante, aumente `limit_margin_deg` no `params.yaml`. No simulador ele parou em 29,3°.
- Anote para o próximo passo (`command_mux`): o sentido dos eixos e se o tilt alcança ±90° com segurança.
