# Ajustes no pantilt_dockerfile para transporte de imagens e câmera

Este documento lista o que mudar no repositório `pantilt_dockerfile` para que as imagens da câmera (`/camera/image_raw` e, depois, `/perception/debug_image`) cheguem na taxa total a todos os nós e à interface web.

O repositório `pantilt_ros` já contém o necessário:
- o perfil do Fast DDS (`pantilt_bringup/config/fastdds.xml`);
- a URL do web_video_server com `qos_profile=sensor_data`.

## Status

Conferido no container em 2026-09-24. Marque aqui ao aplicar cada ajuste no `pantilt_dockerfile`.

- [x] **Ajuste 1** (§2): perfil do Fast DDS em todo o container. A variável está no ambiente do container (PID 1), não no `.bashrc`.
- [x] **Ajuste 2** (§3): `/dev/shm` de 1 GB desde a criação do container.
- [x] **Ajuste 3** (§4): câmera USB. O container roda com todas as capabilities e enxerga `/dev/video*`. O `usbipd attach` continua manual, a cada sessão.
- [x] **Ajuste 4** (§5): aviso do perfil ausente no `/entrypoint.sh`.
- [x] Ajustes manuais antigos removidos (§7): o `/root/.bashrc` voltou ao padrão da imagem.
- [ ] **Ajuste 5** (§6): alias `ws` no Dockerfile. Por enquanto, existe só no container atual.
- [ ] **Ajuste 5, opcional** (§6): `WORKDIR /ros2_ws` no lugar de `/app`.

---

## 1. Problema

O RMW padrão do Humble é o Fast DDS. Entre processos da mesma máquina, ele usa memória compartilhada (SHM), com segmentos de **512 KB** por padrão. Uma imagem 640×480 `bgr8` tem **921 KB**.

Sem espaço no SHM, a mensagem vai por UDP em vários fragmentos. Os tópicos de imagem usam QoS *sensor data* (BEST_EFFORT), sem retransmissão, e basta perder um fragmento para descartar o quadro inteiro.

Medições com a câmera USB entregando ~19 fps, com o perfil `fastdds.xml` (SHM de 10 MB) aplicado em:

| Onde o perfil estava ativo | Taxa recebida em `/camera/image_raw` |
|---|---|
| nenhum processo | 6,7 Hz |
| só no publicador (`camera_node`) | 14 Hz |
| só no assinante | 7 Hz |
| **publicador e assinante** | **19 Hz** (taxa total) |

O mesmo vale para o `web_video_server`: ~5 fps sem o perfil e ~18,5 fps com ele.

**Conclusão:** o perfil precisa valer para **todos os processos ROS do container** (nós, `ros2 launch`, `ros2 topic`, daemon do `ros2`). O lugar certo é uma variável de ambiente do container, e não um launch file.

Custo: cada processo com o perfil reserva o segmento inteiro (~10 MB) em `/dev/shm`. O Docker cria o `/dev/shm` com **64 MB**, que não comporta o sistema completo:

- hardware: bridge e mux;
- percepção: câmera e detector;
- controle: scan e servo;
- gerenciador;
- web: rosbridge e web_video_server;
- daemon e ferramentas de linha de comando.

Quando o `/dev/shm` enche, o Fast DDS volta para UDP e o problema reaparece.

---

## 2. Ajuste 1 (obrigatório): perfil do Fast DDS em todo o container

Defina a variável apontando para o arquivo do repositório `pantilt_ros`, que o entrypoint clona no volume:

```dockerfile
# Dockerfile
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/src/pantilt_ros/pantilt_bringup/config/fastdds.xml
```

ou, se preferir manter no compose:

```yaml
# docker-compose.yml, no serviço do container
    environment:
      - FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/src/pantilt_ros/pantilt_bringup/config/fastdds.xml
```

Com `ENV` no Dockerfile, a variável vale também para os terminais abertos com `docker exec`. No compose, vale igualmente para o `docker exec`, porque é uma variável do container.

Se o arquivo ainda não existir (antes do primeiro clone), o Fast DDS registra `XMLPARSER Error ... loadDefaultXMLFile` e segue com os padrões. Nada quebra, mas as imagens voltam a perder quadros até o repositório estar presente.

---

## 3. Ajuste 2 (obrigatório): aumentar o /dev/shm

```yaml
# docker-compose.yml, no serviço do container
    shm_size: "1gb"
```

Equivalente no `docker run`: `--shm-size=1g`.

Conta de referência: ~10 MB × ~15 processos ROS ≈ 150 MB. O `shm_size` é só um limite: o tmpfs não reserva 1 GB de RAM. O consumo real é o dos segmentos criados, medido em ~10 MB por processo. 1 GB dá folga para ferramentas extras (`ros2 bag`, `rqt`, vários `ros2 topic`).

Alternativa: `ipc: host`, que usa o `/dev/shm` do WSL. Funciona, mas tira o isolamento de IPC do container. Prefira `shm_size`.

---

## 4. Ajuste 3: câmera USB no container

Com o kernel WSL atual (6.18), o driver UVC está presente e a câmera aparece como `/dev/video0`. Ela entrega 640×480 MJPG a até 30 fps; com pouca luz a exposição automática reduz a taxa.

1. **Windows (PowerShell, como administrador)**, antes de subir o container, da mesma forma que a ESP32:
   ```powershell
   usbipd list                                  # anote o BUSID da câmera
   usbipd bind --busid <BUSID>                  # só na primeira vez
   usbipd attach --wsl --busid <BUSID>
   ```
   Se houver script de anexação dos dispositivos, inclua a câmera nele.

2. **Container:** hoje ele já enxerga `/dev/video*`, porque roda com todas as capabilities, provavelmente `privileged: true`. Mantenha assim ou, para restringir, declare os dispositivos:
   ```yaml
       devices:
         - /dev/video0:/dev/video0
   ```
   Nesse caso a câmera precisa estar anexada **antes** de o container subir. Com `privileged`, uma câmera anexada depois também aparece.

3. **Conferência dentro do container:**
   ```bash
   ls -l /dev/video*        # /dev/video0 (captura) e /dev/video1 (metadados, não abre)
   ```

O acesso por URL (stream MJPEG servido no Windows) continua disponível no `camera_node` (`source: "http://..."`), para máquinas cujo kernel WSL não tenha UVC.

---

## 5. Ajuste 4 (opcional): aviso no entrypoint

Depois do bloco que clona o `pantilt_ros`, um aviso ajuda a perceber a falta do perfil:

```bash
if [ -n "$FASTRTPS_DEFAULT_PROFILES_FILE" ] && [ ! -f "$FASTRTPS_DEFAULT_PROFILES_FILE" ]; then
    echo "[entrypoint] AVISO: perfil do Fast DDS não encontrado em $FASTRTPS_DEFAULT_PROFILES_FILE."
    echo "[entrypoint]        Imagens grandes podem perder quadros (ver pantilt_ros/docs/ajustes_pantilt_dockerfile.md)."
fi
```

---

## 6. Ajuste 5: alias `ws` e diretório inicial

O `docker exec -it ptu_web_bridge bash` abre em `/app` (o `WORKDIR` do Dockerfile), sem o ambiente ROS carregado. O alias `ws` entra no workspace e faz os `source` na ordem certa: primeiro o underlay (`/opt/ros/humble`), depois o `install/` do workspace, se existir.

1. Crie o arquivo `bash_aliases` na raiz do repositório `pantilt_dockerfile`:
   ```bash
   # Workspace ROS 2 do pan-tilt: entra em /ros2_ws e carrega o ambiente
   alias ws='cd /ros2_ws && source /opt/ros/humble/setup.bash && if [ -f install/setup.bash ]; then source install/setup.bash; else echo "[ws] install/ ausente: rode colcon build --symlink-install e depois ws"; fi'
   ```

2. Copie-o no Dockerfile. O `.bashrc` padrão do root já carrega o `~/.bash_aliases`:
   ```dockerfile
   COPY bash_aliases /root/.bash_aliases
   ```
   O `COPY` evita escapar as aspas do alias num `RUN echo`.

3. Opcional: se o `/app` não tiver outro uso, troque o diretório inicial:
   ```dockerfile
   WORKDIR /ros2_ws
   ```

Não acrescente `source` ao `/root/.bashrc`: ele fica no padrão da imagem, e o ambiente é carregado explicitamente com `ws`. Depois de um `colcon build`, rode `ws` de novo para carregar pacotes novos.

Uso:
```bash
docker exec -it ptu_web_bridge bash
ws
```

---

## 7. Estado atual do container (temporário)

Os ajustes 1 a 4 já estão no `pantilt_dockerfile`. O `mount -o remount` do `/dev/shm` e o `export` no `/root/.bashrc`, que eram os ajustes manuais anteriores, não são mais necessários, e o `/root/.bashrc` voltou ao padrão da imagem.

Resta um ajuste manual, que existe só no container atual:

| Ajuste manual | Some quando |
|---|---|
| `/root/.bash_aliases` com o alias `ws` (§6) | o container é **recriado** |

Se o container for recriado antes do ajuste 5, recrie o arquivo com o conteúdo da §6. Não coloque `source` no `/root/.bashrc`.

---

## 8. Verificação depois do rebuild

```bash
docker exec -it ptu_web_bridge bash
pwd                                       # /ros2_ws, se o WORKDIR foi trocado
cat ~/.bash_aliases                       # alias ws (ajuste 5)
echo $FASTRTPS_DEFAULT_PROFILES_FILE      # caminho do fastdds.xml
df -h /dev/shm                            # Size 1.0G
ls /sys/class/video4linux/                # video0 e video1, com a câmera anexada

ws
colcon build --symlink-install && ws
ros2 daemon stop                          # o daemon reinicia com o perfil no próximo comando
```

O teste completo de câmera e web está em `docs/teste_camera_web.md`. Com tudo rodando, `df -h /dev/shm` deve mostrar algumas dezenas de MB usados, longe de 1 GB.

---

## 9. Alternativa considerada: Cyclone DDS

Trocar o RMW para o Cyclone DDS (`ros-humble-rmw-cyclonedds-cpp` e `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`) também resolve a perda por fragmentação na mesma máquina. Não foi adotado agora:

- exige um pacote apt a mais;
- todos os processos precisam usar o mesmo RMW;
- muda um componente já validado com o `pantilt_hardware` e o `pantilt_web`.

Fica como opção se o Fast DDS voltar a dar problema.
