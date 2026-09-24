// Ponto de entrada da interface web do pan-tilt.

import * as ros from './ros.js';
import * as diagnostics from './diagnostics.js';
import * as telemetry from './telemetry.js';
import * as manual from './manual.js';
import * as inspection from './inspection.js';
import * as video from './video.js';
import { setGroupEnabled } from './ui.js';

setGroupEnabled('ros', false);

diagnostics.init();
telemetry.init();
manual.init();
inspection.init();
video.init();

ros.start();
