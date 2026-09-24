// Telemetria dos eixos a partir de /joint_states (rad, rad/s -> °, °/s).

import { CONFIG, RAD_TO_DEG } from './config.js';
import { onConnect, onDisconnect, topic } from './ros.js';
import { formatDeg } from './ui.js';

// Sem /joint_states por mais que isso, a telemetria é marcada como antiga
const STALE_MS = 1000;

const els = {};
let lastMsgAt = 0;

export function init() {
  for (const axis of ['pan', 'tilt']) {
    els[axis] = {
      pos: document.getElementById(`${axis}-pos`),
      vel: document.getElementById(`${axis}-vel`),
    };
  }
  const panel = document.getElementById('telemetry');

  onConnect((ros) => {
    topic(ros, CONFIG.topics.jointStates, { throttle_rate: 50 }).subscribe(onJointStates);
  });
  onDisconnect(() => { lastMsgAt = 0; });

  setInterval(() => {
    panel.classList.toggle('stale', performance.now() - lastMsgAt > STALE_MS);
  }, 250);
}

function onJointStates(msg) {
  lastMsgAt = performance.now();
  msg.name.forEach((name, i) => {
    const axis = name === 'pan_joint' ? 'pan' : name === 'tilt_joint' ? 'tilt' : null;
    if (!axis) return;
    if (msg.position.length > i) els[axis].pos.textContent = formatDeg(msg.position[i] * RAD_TO_DEG);
    if (msg.velocity.length > i) els[axis].vel.textContent = formatDeg(msg.velocity[i] * RAD_TO_DEG);
  });
}
