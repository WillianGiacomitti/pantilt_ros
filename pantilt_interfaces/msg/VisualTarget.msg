# Alvo selecionado pelo detector_node (maior score = confianca x area normalizada).
# Publicado em /perception/target a cada frame processado, somente com filtro
# ativo (/perception/set_target). Sem filtro, o topico nao e publicado.
# O header e o da imagem de origem (instante da captura).
# Convencao de erro: centro da bbox menos centro da imagem, em pixels.
#   error_x > 0 -> alvo a direita do centro
#   error_y > 0 -> alvo abaixo do centro

std_msgs/Header header

bool detected            # false quando nenhum candidato valido no frame
string equipment         # chave do equipment.yaml do filtro ativo
float32 confidence

float32 center_x         # px
float32 center_y         # px
float32 width            # px
float32 height           # px

float32 error_x          # px
float32 error_y          # px

uint32 image_width
uint32 image_height
