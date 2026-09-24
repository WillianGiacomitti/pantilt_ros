# Diagnóstico: telemetria de posição errada (encoders)

Registro da investigação de 24/09/2026. A varredura (`/control/scan`) foi testada no pan-tilt real e levou os dois eixos ao fim de curso. A causa está na telemetria de posição enviada pelo firmware, não no `scan_node`. Este documento guarda as evidências, o impacto no `pantilt_ros` e o que precisa estar correto no `pantilt_firmware` antes de voltar aos testes em malha fechada.

---

## 1. Sintoma

- Ao enviar `/control/scan`, o pan-tilt vai ao fim de curso nos dois eixos e fica forçando o batente.
- O `/joint_states` parece "zerado", mesmo com o jog pela página movendo os eixos.

## 2. Evidências

Amostra de 15 s de `/joint_states`, só assinando o tópico, sem nenhum comando em `/ptu/cmd_vel` no período:

| Junta | Posição | Velocidade |
|---|---|---|
| pan | **0,000° em todas as 269 mensagens** | 0 |
| tilt | entre 8,8° e 11,7° (parado) | 10–13°/s, as duas últimas em 10,1°/s constantes |

- A telemetria chega a ~18 Hz; o transporte (serial, bridge, DDS) está funcionando.
- O `/ptu/errors` só registrou `Conectado a /dev/ttyUSB0`: nenhum boot, erro ou perda de dados da ESP32.
- O pan foi levado ao batente pelo jog, e a telemetria continuou em 0,000°. **A posição enviada vem dos encoders (AS5600, lidos em modo analógico), não da contagem de passos.**
- O autor confirmou: um encoder queimou (o do pan) e o outro (o do tilt) oscila cerca de ±2°.
- **Ponto em aberto:** a velocidade do tilt reportada em 10,1°/s constante, sem comandos. Ruído derivado do encoder costumaria variar. Pode ser o firmware ainda aplicando a velocidade da varredura (feita a 10°/s), com o motor forçando o batente. Verificar no firmware de onde vem o campo de velocidade da telemetria.

Os logs dos nós ficam em `~/.ros/log/python3_<pid>_*.log` (o log do `scan_node` registrou as posições de início: pan 0,0° nas duas tentativas; tilt 0,2° e depois -50,4°).

## 3. Por que isso quebra o `pantilt_ros`

Tudo que é malha fechada usa o `/joint_states` como verdade:

| Componente | Uso da posição | Efeito com o pan em 0° e o tilt ruidoso |
|---|---|---|
| `serial_bridge_node` | limites de software (±29° no pan, ±89° no tilt) | nunca dispara no pan: o eixo (por varredura ou jog) chega ao batente mecânico |
| `scan_node` | alvos da varredura | o primeiro trecho (pan até -28°) nunca termina, e o pan é comandado até o batente |
| `scan_node` | detecção de eixo parado (3 s) | não dispara: o ruído do tilt conta como "movimento" |
| `scan_node` | eixo fora do trecho é mantido no alvo | o tilt "corrige" o ruído, e com uma leitura que não acompanha o movimento real a correção empurra sempre para o mesmo lado |
| `visual_servo_node` (futuro) | não usa a posição, mas o scan e o gerenciador dependem dele | — |

Restam como proteção só o fail-safe do firmware (perda de comunicação) e o watchdog do `command_mux` (nó travado). Nenhum dos dois cobre sensor errado.

## 4. Melhorias pendentes no `scan_node` (fazer depois do firmware)

A falha de sensor expôs pontos fracos que valem a pena corrigir mesmo com encoders bons:

1. A detecção de eixo parado deve medir o progresso **do eixo comandado** em direção ao alvo, e não movimento de qualquer eixo.
2. O eixo que não faz parte do trecho deve receber velocidade zero, em vez de corrigir ruído.
3. A tolerância de chegada (0,5°) deve ser maior que o ruído de leitura, ou a leitura deve ser filtrada.
4. Detectar sensor travado: leitura exatamente constante enquanto há velocidade comandada. Avaliar a mesma checagem no `serial_bridge_node`, que poderia avisar em `/ptu/errors` e recusar velocidades.

## 5. Como investigar

### Pelo lado do ROS

```bash
ws
ros2 launch pantilt_bringup hardware.launch.py
# a CLI demora alguns segundos para subir neste ambiente: não conclua nada por timeouts curtos
ros2 topic echo /joint_states --field position     # [pan, tilt] em rad
ros2 topic echo /joint_states --field velocity
ros2 topic echo /ptu/cmd_vel --field angular       # o que realmente chega ao bridge
ros2 topic echo /ptu/errors                        # diagnósticos da ESP32
```

Testes, sempre com velocidades baixas e o STOP da página à mão:

| # | Teste | O que deve acontecer com a telemetria correta |
|---|---|---|
| 1 | Parado por 30 s | posição estável: variação menor que ~0,2° |
| 2 | Jog curto no pan (+ e depois −) | a posição do pan acompanha o movimento e volta |
| 3 | Jog curto no tilt | idem no tilt; o pan não muda |
| 4 | Ir para posição 10° e voltar a 0° | a telemetria chega a ~10° e volta a ~0° |
| 5 | Velocidade com eixo parado | ~0 rad/s em repouso |

**Não use `/control/scan` nem nada em malha fechada** antes de os testes 1 a 5 passarem nos dois eixos.

### Pelo lado do firmware

O `pantilt_firmware` recebeu no CLAUDE.md dele as tarefas correspondentes: origem da telemetria, encoders, auto home e aquecimento do tilt. Resumo do que precisa ser respondido lá:

- de onde vem cada campo do `MSG_TELEMETRY` (encoder ou contagem de passos), para posição e para velocidade;
- se o encoder falhar, o que o firmware envia (hoje, um valor falso e plausível);
- por que o tilt reportou 10°/s constantes sem comandos.

## 6. Critério para voltar aos testes em malha fechada

- Os testes 1 a 5 acima passam nos dois eixos.
- Uma falha de encoder aparece de forma explícita (erro em `/ptu/errors` ou telemetria marcada como inválida), e nunca como um ângulo plausível. Se isso exigir mudar o protocolo serial, o `serial_bridge_node` e o `docs/architecture.md` §8 mudam junto.
- Depois disso: aplicar as melhorias da seção 4, testar a varredura no hardware e seguir com a coleta de vídeos (Etapa 2, `capture_node`).
