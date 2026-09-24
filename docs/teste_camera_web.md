# Roteiro de teste da câmera com a interface web

## Contexto

Valida o item "pantilt_web: testar com a camera" do CLAUDE.md. O caminho testado é `camera_node` → `/camera/image_raw` → `web_video_server` → página. A última parte acrescenta o hardware: com o jog da página, a imagem deve se mover junto com o pan-tilt. Não há mudança de código; são só comandos para você executar.

Enquanto o `detector_node` não existe, a página mostra `/camera/image_raw` no lugar do `/perception/debug_image`, via `?video_topic=`.

Endereços (no navegador do Windows):

| O quê | URL |
|---|---|
| Página | `http://localhost:8000/?video_topic=/camera/image_raw` |
| Stream direto | `http://localhost:8080/stream?topic=/camera/image_raw&qos_profile=sensor_data` |
| Tópicos que o web_video_server enxerga | `http://localhost:8080/` |

O `qos_profile=sensor_data` é obrigatório no stream direto. A página já o inclui (`web/js/config.js`).

---

## 1. Antes de subir o container (Windows)

```powershell
usbipd list                              # anote o BUSID da câmera (e da ESP32, para a seção 6)
usbipd attach --wsl --busid <BUSID>
```

## 2. Conferência do ambiente (terminal 1, dentro do container)

```bash
docker exec -it ptu_web_bridge bash
ws                                        # entra em /ros2_ws e carrega o ambiente

echo $FASTRTPS_DEFAULT_PROFILES_FILE      # .../pantilt_bringup/config/fastdds.xml
df -h /dev/shm                            # Size 1.0G
ls /sys/class/video4linux/                # video0 e video1: câmera anexada de fato
ps aux | grep -v grep | grep -E 'camera_node|web_video_server'   # nenhum processo antigo

colcon build --symlink-install && ws
ros2 daemon stop                          # o daemon reinicia com o perfil do Fast DDS
```

O `ls /dev/video*` sozinho não basta: o nó `/dev/video0` pode continuar existindo depois que a câmera foi desanexada. O `/sys/class/video4linux/` mostra o estado real.

## 3. Câmera (terminal 1)

Até existir o `perception.launch.py`, o nó sobe com `ros2 run`:

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

## 4. Verificação ROS (terminal 2)

Em todo terminal novo: `docker exec -it ptu_web_bridge bash` e `ws`.

```bash
ros2 param dump /camera_node                # confere que o params.yaml foi lido
ros2 topic hz /camera/image_raw             # ~19-30 Hz conforme a luz; ~7 Hz indica problema de transporte (seção 8)
ros2 topic info -v /camera/image_raw        # publisher camera_node: Reliability BEST_EFFORT
ros2 topic echo /camera/image_raw --field header   # stamp avançando e frame_id camera_optical_frame
```

Anote a taxa desta etapa: ela é a referência para a seção 7.

## 5. Web (terminal 3)

```bash
ros2 launch pantilt_web web.launch.py
```

Esperado no log: `Waiting For connections on 0.0.0.0:8080` (web_video_server) e `Rosbridge WebSocket server started on port 9090`.

Abra a página: `http://localhost:8000/?video_topic=/camera/image_raw`.

- O painel mostra `Aguardando imagens` e, em seguida, o vídeo.
- O código abaixo do painel mostra o tópico `/camera/image_raw`.
- O log do rosbridge mostra `Client connected`.

No terminal 2, confira a assinatura criada pela página:

```bash
ros2 topic info -v /camera/image_raw        # subscription web_video_server: Reliability BEST_EFFORT
```

Se a assinatura aparecer como **RELIABLE**, o stream foi aberto sem `qos_profile=sensor_data` e não recebe quadros (seção 8).

## 6. Vídeo com o hardware (terminal 4)

Com a ESP32 anexada:

```bash
ros2 launch pantilt_bringup hardware.launch.py
```

Esperado no log: `Limites efetivos: pan [-29.0°, 29.0°], tilt [-89.0°, 89.0°]`, `Conectado a /dev/ttyUSB0 @ 921600 bps`, `Prioridade do operador: 1.00 s | watchdog de comando: 0.30 s` e `Fonte de controle inicial: none`.

Pela página, com velocidades baixas no jog:

| # | Ação | Esperado |
|---|---|---|
| 1 | Segurar ► (pan positivo) | a imagem desliza no sentido do movimento; o indicador **Fonte** mostra `web` |
| 2 | Soltar o botão | o eixo para na hora (a página publica velocidade zero ao soltar) |
| 3 | Segurar ▲ (tilt positivo) | a imagem sobe ou desce de forma coerente; anote o sentido |
| 4 | STOP com o eixo em movimento | parada imediata |
| 5 | Mover o pan até o limite | o eixo para perto de 29° e a imagem congela no enquadramento do limite |

Anote o sentido da imagem em cada eixo. É isso que define o sinal do erro do `visual_servo_node` (`invert_pan`/`invert_tilt`).

Ao encerrar o launch com Ctrl+C, pode aparecer um `KeyboardInterrupt` no traceback do `serial_bridge_node`. O `ros2 launch` repassa um segundo SIGINT enquanto o nó já está encerrando. A velocidade zero é enviada antes desse ponto (`destroy_node`), então o eixo para mesmo assim.

## 7. Testes

| # | Teste | Como | Esperado |
|---|---|---|---|
| 1 | Taxa com o navegador | página aberta; `ros2 topic hz /camera/image_raw` | mesma taxa da seção 4 |
| 2 | Duas abas | abra a página numa segunda aba | as duas mostram vídeo; `ros2 topic info -v` lista uma assinatura BEST_EFFORT por stream aberto; anote se a taxa cai |
| 3 | Latência visual | passe a mão rápido diante da câmera | anote o atraso percebido |
| 4 | Parar a câmera | Ctrl+C no terminal 1 | o vídeo congela no último quadro; anote se a página muda para `Aguardando imagens` |
| 5 | Voltar a câmera | suba o `camera_node` de novo | anote se o vídeo volta sozinho ou só ao recarregar a página |
| 6 | Reiniciar a web | Ctrl+C no terminal 3 e suba o `web.launch.py` de novo | a página reconecta ao rosbridge e pede o stream de novo (`video.js`, `onConnect`); o vídeo volta sem recarregar |
| 7 | Câmera desconectada | `usbipd detach --busid <BUSID>` no Windows | log `Fonte "0" parou de entregar quadros; reabrindo` e depois `Não foi possível abrir a fonte "0"` a cada 5 s |
| 8 | Câmera reconectada | `usbipd attach --wsl --busid <BUSID>` | anote se o nó volta a abrir a fonte sozinho (`Fonte aberta: ...`) ou se precisa ser reiniciado |

Para registrar (opcional): `ros2 bag record /camera/image_raw`. Ocupa ~20 MB/s a 640×480 e 20 Hz.

## 8. Solução de problemas

| Sintoma | Causa provável | O que fazer |
|---|---|---|
| `offering incompatible QoS ... RELIABILITY_QOS_POLICY` no log do web_video_server | stream aberto sem `qos_profile=sensor_data` | use a URL da página ou inclua o parâmetro. As assinaturas RELIABLE antigas continuam listadas até reiniciar o `web.launch.py` |
| `Não foi possível abrir a fonte "0"` repetido | câmera não anexada ao WSL (o `/dev/video0` pode existir mesmo assim) | `ls /sys/class/video4linux/`; se estiver vazio, refaça o `usbipd attach` |
| `ros2 topic hz` em ~7 Hz ou vídeo em ~5 fps | processo sem o perfil do Fast DDS ou `/dev/shm` cheio | `echo $FASTRTPS_DEFAULT_PROFILES_FILE` no terminal do nó; `df -h /dev/shm`; `ros2 daemon stop` |
| Página em `Servidor de vídeo indisponível` | web_video_server fora do ar ou em outra porta | confira o terminal 3; com outra porta, abra a página com `?video=<porta>` |
| Tópico não encontrado no stream | tópico com `%2F` na URL | use `/` literal: o web_video_server não decodifica `%2F` |
| `ros2 param dump` com valores diferentes do yaml | nó iniciado sem `--ros-args --params-file` (ex.: `--ros_param`, que é ignorado) | suba de novo com o comando da seção 3 |
| `ros2: command not found` ou pacote não encontrado | terminal sem o ambiente | `ws`; depois de um `colcon build`, `ws` de novo |

## Observações

- O `detector_node` vai publicar `/perception/debug_image`, que é o tópico padrão da página. Depois dele, abra a página sem `?video_topic=`.
- Com o sistema todo rodando, `df -h /dev/shm` deve mostrar algumas dezenas de MB usados, longe de 1 GB.
