# Estado do capture_node. Publicado em /capture/status a 2 Hz e a cada mudanca
# (QoS transient_local: a interface recebe o ultimo estado ao conectar).

std_msgs/Header header

bool recording
bool scanning            # true enquanto um goal Scan esta ativo
string session           # nome da sessao limpo (vazio fora de gravacao)
string file              # video em gravacao ou o ultimo gravado
float32 elapsed_s        # duracao da gravacao atual
uint32 frames            # quadros gravados
uint32 dropped           # quadros descartados por fila cheia
string message           # texto legivel para a interface
