# Estado do experiment_recorder (docs/architecture.md, secao 4.11). Publicado em
# /experiment/status a 2 Hz e a cada mudanca (QoS transient_local).

std_msgs/Header header

bool recording
string label               # rotulo do ensaio (limpo para [a-z0-9_.-])
uint32 repetition          # da gravacao atual ou a proxima, se parado
string notes               # observacao livre
string image_mode          # none, snapshots ou video
string folder              # pasta da gravacao atual ou a ultima gravada
float32 elapsed_s          # duracao da gravacao atual
string message             # texto legivel para a interface
