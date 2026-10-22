# Tempos do detector_node por quadro processado (docs/architecture.md, secao 4.2).
# Publicado em /perception/timing; o header e o da imagem (instante da captura).

std_msgs/Header header

float32 inference_ms       # tempo de inferencia da YOLO
float32 latency_ms         # da captura ao fim do processamento do quadro (relogio do host)
uint32 n_detections        # deteccoes publicadas (apos o filtro)
