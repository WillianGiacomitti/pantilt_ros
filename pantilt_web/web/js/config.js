// Configuração da interface. Nomes de tópicos e services seguem
// docs/architecture.md (seção 5). Graus só existem aqui na web: os tópicos
// são em rad e rad/s.

const query = new URLSearchParams(window.location.search);
const host = window.location.hostname || 'localhost';

export const RAD_TO_DEG = 180 / Math.PI;
export const DEG_TO_RAD = Math.PI / 180;

export const CONFIG = {
  // Portas padrão do web.launch.py; sobrescreva com ?ws=9090&video=8080
  rosbridgeUrl: `ws://${host}:${query.get('ws') || 9090}`,
  videoBaseUrl: `http://${host}:${query.get('video') || 8080}`,

  reconnectMs: 2000,         // espera entre tentativas de reconectar ao rosbridge
  videoRetryMs: 5000,        // nova tentativa do stream de vídeo

  // Novas tentativas enquanto um nó ainda não existe (reassinatura de tópicos
  // transient_local e /inspection/list_equipment). O intervalo dobra a cada
  // tentativa até o máximo, para não encher o log do rosbridge.
  retryMinMs: 3000,
  retryMaxMs: 10000,

  // Repetição do jog enquanto o botão está pressionado. Precisa ser menor que
  // cmd_timeout_s do command_mux (0,3 s), senão o watchdog zera a velocidade.
  jogRepeatMs: 100,

  jog: {
    minDegS: 1,
    maxDegS: 90,
    panDefaultDegS: 30,
    tiltDefaultDegS: 20,
  },

  // Limites físicos (serial_bridge_node: pan_limits_deg / tilt_limits_deg).
  // Só restringem os campos da página; o corte real é feito no bridge.
  limitsDeg: {
    pan: [-30, 30],
    tilt: [-90, 90],
  },

  video: {
    // Imagem com as bboxes do detector_node. Para testar só a câmera, abra a
    // página com ?video_topic=/camera/image_raw
    topic: query.get('video_topic') || '/perception/debug_image',
    type: 'mjpeg',
    // Os tópicos de imagem usam QoS sensor data (BEST_EFFORT); sem isto o
    // web_video_server assina como RELIABLE e não recebe nenhum quadro
    qos: 'sensor_data',
  },

  topics: {
    jointStates:   { name: '/joint_states',           type: 'sensor_msgs/JointState' },
    cmdVelWeb:     { name: '/ptu/cmd_vel_web',        type: 'geometry_msgs/Twist' },
    cmdPosWeb:     { name: '/ptu/cmd_pos_web',        type: 'sensor_msgs/JointState' },
    controlSource: { name: '/ptu/control_source',     type: 'std_msgs/String' },
    errors:        { name: '/ptu/errors',             type: 'std_msgs/String' },
    status:        { name: '/inspection/status',      type: 'pantilt_interfaces/InspectionStatus' },
    detections:    { name: '/perception/detections',  type: 'vision_msgs/Detection2DArray' },
  },

  services: {
    setZero:       { name: '/ptu/set_zero',               type: 'std_srvs/Trigger' },
    listEquipment: { name: '/inspection/list_equipment',  type: 'pantilt_interfaces/ListEquipment' },
    start:         { name: '/inspection/start',           type: 'pantilt_interfaces/StartInspection' },
    stop:          { name: '/inspection/stop',            type: 'std_srvs/Trigger' },
  },
};

// Constantes de pantilt_interfaces/srv/StartInspection
export const MODE_CENTER = 0;
export const MODE_TRACK = 1;
