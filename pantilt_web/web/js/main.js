// Ponto de entrada da interface web do pan-tilt.

import { CONFIG } from './config.js';
import * as ros from './ros.js';
import * as diagnostics from './diagnostics.js';
import * as telemetry from './telemetry.js';
import * as manual from './manual.js';
import * as inspection from './inspection.js';
import * as capture from './capture.js';
import * as video from './video.js';
import { setGroupEnabled, initTabs } from './ui.js';

setGroupEnabled('ros', false);

diagnostics.init();
telemetry.init();
manual.init();
inspection.init();
capture.init();

// Abas Inspeção | Coleta. Na Coleta, o vídeo mostra a imagem crua da câmera,
// que é o que o capture_node grava.
initTabs(document.getElementById('insp-panel'), CONFIG.initialTab, (tab) => {
  video.setTopic(tab === 'coleta' ? CONFIG.video.rawTopic : CONFIG.video.topic);
});
video.init();

ros.start();
