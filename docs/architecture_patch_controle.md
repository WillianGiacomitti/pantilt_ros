# Patch do `docs/architecture.md` — camada de controle e calibração

Este arquivo contém as seções a **inserir ou substituir** no `docs/architecture.md`. Ele não substitui o documento inteiro, porque o original já evoluiu (v0.3). Aplique seção por seção e suba a versão do documento para **v0.4 (08/10/2026)**.

Resumo das mudanças:

| Seção | Ação |
|---|---|
| §4.1 `camera_node` | acrescentar a publicação de `/camera/camera_info` |
| §4.5 `visual_servo_node` | substituir pela especificação completa abaixo |
| §4.6 `command_mux` | acrescentar `/ptu/cmd_pos_auto` |
| §4.9 `calibration_node` | **nova seção** |
| §5.1 Tópicos | acrescentar duas linhas |
| §5.2 Services | acrescentar duas linhas |
| §9 Segurança | acrescentar duas linhas |
| §10 Métricas | precisar as definições de `t_c` e `e_r` |

---

## §4.1 `camera_node` — acrescentar ao final da seção

Além da imagem, o nó publica os parâmetros intrínsecos da câmera em `/camera/camera_info` (`sensor_msgs/CameraInfo`), lidos de um arquivo no formato padrão do ROS (`camera_info_manager`). É assim que `f_x`, `f_y`, `c_x` e `c_y` chegam ao `visual_servo_node`; esses valores nunca são parâmetros do nó de controle nem constantes no código.

| Parâmetro | Padrão | Descrição |
|---|---|---|
| `camera_info_file` | `config/camera_intrinsics.yaml` | Arquivo gerado pelo `calibration_node` |
| `camera_name` | `pantilt_cam` | Nome usado no arquivo de calibração |

Regras:

- a mensagem de `CameraInfo` usa o **mesmo `header.stamp` e `frame_id`** do quadro correspondente, e vai no mesmo perfil de QoS da imagem;
- se o arquivo não existir, o nó publica `CameraInfo` com a matriz `K` zerada e registra um aviso único no log. Quem consome é responsável por tratar `f_x = 0` como "sem calibração";
- se a resolução configurada no nó não bater com a do arquivo, o nó avisa e **não** reescala os valores: a calibração deve ser refeita.

---

## §4.5 `visual_servo_node` (pantilt_control) — substituir a seção

Servidor da action `/control/center`. Implementa a malha IBVS: converte o erro de imagem em velocidade angular dos eixos.

### Fundamento

A lei clássica de IBVS, `v_c = −λ·L⁺·e`, é uma ação proporcional aplicada após o mapeamento geométrico dado pela pseudo-inversa da matriz de interação (CHAUMETTE; HUTCHINSON, 2006). Como o pan-tilt executa rotação pura, as colunas da matriz de interação que dependem da profundidade `Z` desaparecem, e `L⁺` degenera na associação direta entre cada componente do erro e um eixo. Escrevendo o erro como ângulo de linha de visada, `θ = atan(e_px / f)`, o fator `(1 + x²)` da matriz de interação é cancelado de forma exata, porque ele é a derivada da tangente. O resultado é uma planta integradora de ganho unitário em cada eixo, e o controlador se reduz a dois PIDs escalares. Dedução completa na seção teórica da monografia.

### Pipeline (cinco estágios)

1. **Conversão.** `θ_pan = atan(error_x / f_x)` e `θ_tilt = atan(error_y / f_y)`, com `f_x` e `f_y` vindos de `/camera/camera_info`. Se ainda não houve `CameraInfo` válido (`f_x > 0`), o goal é **rejeitado** com mensagem explícita.
2. **Controlador.** Duas instâncias independentes criadas pela fábrica `criar_controlador(nome, params)` de `controllers/base.py`, segundo o parâmetro `controller`. A interface é `compute(erro, dt) -> velocidade`. O `dt` é calculado pelos `header.stamp` das mensagens de alvo, não pelo relógio do laço, porque a taxa do detector varia.
3. **Saturação e anti-windup.** Saída limitada a `max_vel_deg_s`; o integrador é congelado enquanto a saída está saturada e o termo integral tem teto próprio (`integral_limit_deg_s`).
4. **Supervisão.** Avalia três condições de término e calcula as métricas:
   - **centrado:** `error_px ≤ tolerance_px` continuamente por `hold_time_s`;
   - **perdido:** sem alvo (`detected == false` ou sem mensagem) por `lost_timeout_s`;
   - **preso no limite:** eixo no limite de software do bridge, com comando empurrando para fora, por mais de `limit_timeout_s`.
5. **Publicação.** Publica em `/ptu/cmd_vel_auto` a `publish_rate_hz`, repetindo o último comando calculado por até `cmd_hold_s`. Passado esse tempo sem alvo novo, publica zero. Isso desacopla a taxa do detector (~10 Hz) do watchdog de 0,3 s do `command_mux` e evita movimento aos trancos.

### Semântica da action

| Situação | Resultado |
|---|---|
| Modo `MODE_CENTER`, centrado | `success=true`, com `final_error_px` e `convergence_time_s` |
| Modo `MODE_TRACK`, centrado | a action **continua ativa**; o feedback passa a trazer `centered=true` |
| Alvo perdido | `success=false`, mensagem "alvo perdido"; velocidade zerada antes de abortar |
| Preso no limite | `success=false`, mensagem indicando o eixo e o limite atingido |
| Goal cancelado | velocidade zerada; resultado com `success=false` e mensagem "cancelado" |
| Novo goal | substitui o anterior (mesmo padrão do `scan_node`); os controladores são reiniciados com `reset()` |

Em qualquer término, o nó publica velocidade zero antes de responder.

### Parâmetros

| Parâmetro | Padrão | Unidade | Descrição |
|---|---|---|---|
| `controller` | `pid` | — | `pid` ou `fuzzy` |
| `pan.kp` / `tilt.kp` | 0.8 | s⁻¹ | equivale ao ganho λ do IBVS; teto teórico ~3 |
| `pan.ki` / `tilt.ki` | 0.0 | s⁻² | ativar só no ensaio com alvo em movimento |
| `pan.kd` / `tilt.kd` | 0.0 | — | ativar só após o PI estável |
| `derivative_filter_hz` | 2.0 | Hz | filtro de 1ª ordem na derivada |
| `max_vel_deg_s` | 20.0 | °/s | saturação da saída |
| `integral_limit_deg_s` | 10.0 | °/s | teto da contribuição integral |
| `invert_pan` / `invert_tilt` | false | — | convenção de sinal, definida no teste de bancada |
| `publish_rate_hz` | 20.0 | Hz | estágio 5 |
| `cmd_hold_s` | 0.25 | s | repetição do último comando |
| `limit_timeout_s` | 2.0 | s | tempo preso no limite antes de abortar |

### Organização do código

```
pantilt_control/
├── visual_servo_node.py          # ROS: action server, assinaturas, publicação
├── visual_servo_math.py          # puro: conversão px→ângulo, critérios, métricas
└── controllers/
    ├── base.py                   # interface + criar_controlador()
    ├── pid.py                    # PID posicional com anti-windup
    └── fuzzy.py                  # (depois) Mamdani, mesma interface
```

A lógica pura fica fora do nó, com testes em pytest, como já foi feito em `scan_pattern.py`. A troca entre PID e fuzzy deve ser apenas o parâmetro `controller`: nenhum outro arquivo muda.

### Sinais

O sentido de giro depende da montagem mecânica. O procedimento de bancada (ganho 0,2 s⁻¹, alvo deslocado para um lado) define `invert_pan` e `invert_tilt`, registrados em `docs/testes.md`.

---

## §4.6 `command_mux` — acrescentar

O mux passa a aceitar também comandos de **posição** pelo caminho automático, em `/ptu/cmd_pos_auto`, encaminhados para `/ptu/cmd_pos`. Isso é necessário para o `calibration_node`, que precisa posicionar os eixos em ângulos conhecidos.

Regras:

- a prioridade do operador vale igualmente: comandos automáticos de posição são descartados enquanto a web tiver publicado nos últimos `operator_hold_s`;
- o watchdog de `cmd_timeout_s` **não** se aplica a comandos de posição, que são um alvo a atingir e não um fluxo contínuo;
- `/ptu/control_source` continua refletindo a fonte ativa (`web`, `auto` ou `none`).

---

## §4.9 `calibration_node` (pantilt_perception) — nova seção

Estima os parâmetros intrínsecos usando o próprio mecanismo: girando a câmera em ângulos conhecidos e medindo o deslocamento do alvo na imagem.

### Fundamento

Pelo modelo pinhole, `u − c_x = f_x · tan(θ)`, em que θ é o ângulo entre o eixo óptico e a linha de visada ao alvo. Sob **rotação pura**, a profundidade do alvo não entra na equação, de modo que a distância até o alvo e o tamanho dele são irrelevantes. Essa é a mesma propriedade que dispensa a estimativa de `Z` no IBVS deste sistema.

Com o alvo fixo no mundo e o eixo em `θ_j`, o modelo com três incógnitas por eixo é:

```
u_i = c_x + f_x · tan(φ_pan − θ_pan,i)
v_i = c_y + f_y · tan(φ_tilt − θ_tilt,i)
```

em que `φ` é a direção (desconhecida) do alvo. Usar várias amostras e ajustar por mínimos quadrados, em vez da fórmula de dois pontos, estima também o ponto principal, dá um resíduo que denuncia encoder ruim ou distorção, e dispensa centralizar o alvo com precisão.

### Interface

| Service | Tipo | Função |
|---|---|---|
| `/calibration/run` | `std_srvs/Trigger` | Executa a rotina completa e grava o arquivo |
| `/calibration/abort` | `std_srvs/Trigger` | Interrompe a rotina e devolve os eixos à posição inicial |

A rotina leva dezenas de segundos, e o `run` responde só ao final, com os valores e o resíduo na mensagem. Por isso o service usa `ReentrantCallbackGroup` com `MultiThreadedExecutor`: sem isso, as assinaturas de `/perception/target` e `/joint_states` ficam paradas durante a execução.

### Pré-condições (verificadas ao receber `run`)

1. `/perception/target` com `detected == true` e erro dentro de 20% da meia-largura do quadro (alvo aproximadamente centralizado pelo operador);
2. eixos parados e próximos de zero (o operador já usou `/ptu/set_zero`);
3. `/ptu/control_source` diferente de `web`.

Falhando qualquer uma, o service retorna `success=false` com a razão. O nó **não** centraliza o alvo sozinho: essa é a etapa manual do operador, feita pela página web.

### Rotina

1. Registra a posição inicial (*home*) a partir de `/joint_states`.
2. **Sondagem:** move o pan `±probe_deg` (padrão 3°), mede o deslocamento e estima um `f_x` grosseiro.
3. **Grade em cruz adaptativa:** com o `f_x` grosseiro, escolhe `n_points` ângulos simétricos cujo deslocamento previsto fique em torno de 35% da meia-largura do quadro, limitados a `max_angle_deg`. Varre o pan com o tilt em zero e depois o tilt com o pan em zero. A cruz evita o acoplamento cruzado entre os eixos e mantém cada ajuste unidimensional.
4. Em cada ponto: envia posição por `/ptu/cmd_pos_auto`, espera a junta estabilizar (`|θ − θ_alvo| < settle_tol_deg` com velocidade próxima de zero), espera `settle_extra_s`, e então coleta `samples_per_point` mensagens de alvo **cujo `header.stamp` seja posterior ao instante de parada** — por causa dos ~115 ms de atraso da imagem. Guarda a mediana de `u` e `v` com o ângulo medido.
5. Pontos sem detecção são descartados, com aviso no log. Menos de `min_points` válidos por eixo aborta a calibração.
6. Retorna ao *home* e publica velocidade zero.
7. Ajusta os dois modelos (Levenberg-Marquardt via `scipy.optimize.least_squares`, já presente como dependência do Ultralytics; chute inicial pela fórmula de dois pontos, com `c` no centro geométrico e `φ = 0`).
8. Grava `config/camera_intrinsics.yaml` no formato padrão do ROS (`image_width`, `image_height`, `camera_name`, `camera_matrix`, `distortion_coefficients` zerados, `rectification_matrix`, `projection_matrix`) e devolve na mensagem `f_x`, `f_y`, `c_x`, `c_y`, o resíduo RMS em px, o número de pontos usados e a comparação com o FOV nominal.

### Parâmetros

| Parâmetro | Padrão | Descrição |
|---|---|---|
| `probe_deg` | 3.0 | Amplitude da sondagem inicial |
| `max_angle_deg` | 10.0 | Ângulo máximo da grade (limita distorção) |
| `n_points` | 7 | Pontos por eixo, simétricos em torno de zero |
| `samples_per_point` | 5 | Mensagens de alvo por ponto (usa a mediana) |
| `settle_tol_deg` | 0.2 | Tolerância para considerar a junta parada |
| `settle_extra_s` | 0.4 | Espera extra após a parada (atraso da imagem) |
| `min_points` | 4 | Mínimo de pontos válidos por eixo |
| `output_file` | `config/camera_intrinsics.yaml` | Arquivo gerado |

### Interpretação do resultado

- resíduo RMS abaixo de ~2 px: bom;
- resíduo alto com padrão sistemático nas pontas da grade: distorção radial; reduza `max_angle_deg` ou faça a calibração por tabuleiro;
- resíduo alto e disperso: provável erro de encoder (ver `docs/diagnostico_encoders.md`);
- `f_x` distante de `(W/2)/tan(FOV_h/2)` em mais de ~15%: desconfie do encoder ou do FOV declarado.

A calibração por tabuleiro de xadrez (OpenCV) continua sendo o método de referência e permanece como script offline, não como nó. Ela é obrigatória se a OAK-D entrar no projeto ou se a distorção se mostrar relevante.

---

## §5.1 Tópicos — acrescentar

| Tópico | Tipo | Publica | Assina | QoS |
|---|---|---|---|---|
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | camera_node | visual_servo_node, calibration_node | igual ao da imagem |
| `/ptu/cmd_pos_auto` | `sensor_msgs/JointState` | calibration_node | command_mux | padrão |

## §5.2 Services — acrescentar

| Nome | Tipo | Cliente → Servidor |
|---|---|---|
| `/calibration/run` | `std_srvs/Trigger` | operador (CLI ou web) → calibration_node |
| `/calibration/abort` | `std_srvs/Trigger` | operador → calibration_node |

## §9 Segurança — acrescentar

| Camada | Mecanismo | Protege contra |
|---|---|---|
| `visual_servo_node` | aborto por permanência no limite de ângulo | malha empurrando contra o batente indefinidamente |
| `calibration_node` | aborto se o operador assumir o controle; retorno ao *home* em qualquer saída | movimento autônomo inesperado durante a calibração |

## §10 Métricas — substituir as definições

| Métrica | Definição operacional |
|---|---|
| `t_c` | intervalo entre a **primeira detecção válida** após o início do goal e o instante em que o erro entra na tolerância e nela permanece por `hold_time_s` |
| `e_r` | **média** de `error_px` durante esse intervalo de permanência (menos sensível ao tremor da bbox que o último valor) |
| `e_d` | média de `error_px` no feedback, em modo TRACK, durante as oscilações da base |