# Estado do inspection_manager. Publicado em /inspection/status
# (QoS transient_local: a interface recebe o ultimo estado ao conectar).

string STATE_IDLE=IDLE
string STATE_SEARCHING=SEARCHING
string STATE_CENTERING=CENTERING
string STATE_TRACKING=TRACKING

std_msgs/Header header

string state             # um dos STATE_* acima
string equipment         # chave do equipamento em inspecao (vazio em IDLE)
bool autonomous          # true enquanto o modo automatico controla o PTU
float32 error_px         # erro euclidiano atual (0 fora de CENTERING/TRACKING)
string message           # texto legivel para a interface
