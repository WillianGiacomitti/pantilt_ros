# Alvo selecionado pelo detector_node (maior score = confianca x area).
# Publicado em /perception/target a cada frame processado.
# Convencao de erro: centro da bbox menos centro da imagem, em pixels.
#   error_x > 0 -> alvo a direita do centro
#   error_y > 0 -> alvo abaixo do centro

std_msgs/Header header

bool detected            # false quando nenhum candidato valido no frame
string equipment         # chave do equipment.yaml (vazio se sem filtro)
float32 confidence

float32 center_x         # px
float32 center_y         # px
float32 width            # px
float32 height           # px

float32 error_x          # px
float32 error_y          # px

uint32 image_width
uint32 image_height
