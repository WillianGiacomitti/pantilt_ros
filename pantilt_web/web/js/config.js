// Configuração da interface. Nomes de tópicos e services seguem
// docs/architecture.md (seção 5). Graus só existem aqui na web: os tópicos
// são em rad e rad/s.

const query = new URLSearchParams(window.location.search);
const host = window.location.hostname || 'localhost';

export const RAD_TO_DEG = 180 / Math.PI;
export const DEG_TO_RAD = Math.PI / 180;

export const CONFIG = {
  // Portas padrão do web.launch.py (as publicadas pelo docker-compose);
  // sobrescreva com ?ws=9090&video=8081
  rosbridgeUrl: `ws://${host}:${query.get('ws') || 9090}`,
  videoBaseUrl: `http://${host}:${query.get('video') || 8081}`,

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

  // Aba aberta ao carregar a página: ?tab=coleta abre o painel de coleta
  initialTab: query.get('tab') === 'coleta' ? 'coleta' : 'inspecao',

  video: {
    // Imagem com as bboxes do detector_node. Para testar só a câmera, abra a
    // página com ?video_topic=/camera/image_raw
    topic: '/perception/debug_image',
    // ?video_topic= fixa o tópico, mesmo ao trocar de aba
    topicOverride: query.get('video_topic'),
    // Imagem crua da câmera: é o que o capture_node grava (aba Coleta)
    rawTopic: '/camera/image_raw',
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
    captureStatus: { name: '/capture/status',         type: 'pantilt_interfaces/CaptureStatus' },
  },

  services: {
    setZero:       { name: '/ptu/set_zero',               type: 'std_srvs/Trigger' },
    listEquipment: { name: '/inspection/list_equipment',  type: 'pantilt_interfaces/ListEquipment' },
    start:         { name: '/inspection/start',           type: 'pantilt_interfaces/StartInspection' },
    stop:          { name: '/inspection/stop',            type: 'std_srvs/Trigger' },
    captureStart:  { name: '/capture/start',              type: 'pantilt_interfaces/StartCapture' },
    captureStop:   { name: '/capture/stop',               type: 'std_srvs/Trigger' },
  },

  capture: {
    // /capture/status chega a 2 Hz; sem ele por mais que isso, o capture_node
    // é considerado fora do ar
    statusStaleMs: 2000,
    // Velocidade máxima aceita pelo capture_node/scan_node
    maxSpeedDegS: 30,
  },
};

// Constantes de pantilt_interfaces/srv/StartInspection
export const MODE_CENTER = 0;
export const MODE_TRACK = 1;
