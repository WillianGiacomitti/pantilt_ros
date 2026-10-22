# Sinais internos da malha IBVS do visual_servo_node (docs/architecture.md, secao 4.5).
# Publicado em /control/servo_debug uma vez por alvo recebido durante um goal
# Center ativo (mesma cadencia do feedback, ~10 Hz), para os ensaios do TCC.
# O header e o do alvo (instante da captura da imagem).
# Com target_visible=false o controlador nao roda: campos do controlador e do
# comando vem zerados.

std_msgs/Header header

string controller          # parametro controller do no (pid, fuzzy)
uint8 mode                 # Center.Goal.mode (0 = CENTER, 1 = TRACK)

# Entrada
bool target_visible
float32 error_x            # px
float32 error_y            # px
float32 error_px           # px, norma do erro
float64 theta_pan          # rad, atan(error_x / f_x)
float64 theta_tilt         # rad, atan(error_y / f_y)

# Controlador
float64 dt                 # s, pelos header.stamp dos alvos; 0 na primeira amostra
bool gap_reset             # controladores reiniciados por lacuna entre alvos
float64 p_pan              # rad/s, termos do PID (zerados em controladores sem P/I/D)
float64 i_pan
float64 d_pan
float64 p_tilt
float64 i_tilt
float64 d_tilt
float64 u_raw_pan          # rad/s, saida antes da saturacao
float64 u_raw_tilt
bool saturated_pan
bool saturated_tilt

# Saida: comando calculado apos saturacao e invert_* (o publicado a 20 Hz
# continua em /ptu/cmd_vel_auto)
float64 cmd_pan            # rad/s
float64 cmd_tilt           # rad/s

# Supervisao: metricas da primeira convergencia (0 se ainda nao houve)
bool centered
float32 convergence_time_s
float32 final_error_px
