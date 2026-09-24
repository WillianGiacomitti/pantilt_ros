// Controle manual: jog por velocidade, ir para posição, zerar e parar.
//
// Jog: enquanto o botão estiver pressionado, /ptu/cmd_vel_web é republicado a
// cada CONFIG.jogRepeatMs. O command_mux zera a velocidade se a fonte ficar
// em silêncio por mais de cmd_timeout_s (0,3 s), então uma publicação única
// não basta. Ao soltar, sair do botão, trocar de aba ou perder a conexão, o
// jog para. Sem conexão nada é publicado; o watchdog do mux zera o eixo.
//
// Convenção do Twist para o PTU: angular.z = pan, angular.y = tilt (rad/s).

import { CONFIG, DEG_TO_RAD } from './config.js';
import { onConnect, onDisconnect, isConnected, topic, service, callService } from './ros.js';
import { toast, setGroupEnabled } from './ui.js';

let velPub = null;
let posPub = null;
let setZeroSrv = null;

let activeBtn = null;
let jogTimer = null;

const sliders = {};

export function init() {
  initSliders();
  initJogButtons();
  initPosition();

  document.getElementById('btn-stop').addEventListener('pointerdown', (e) => {
    e.preventDefault();
    stopAll();
  });
  document.getElementById('btn-stop-all').addEventListener('click', stopAll);
  document.getElementById('btn-zero').addEventListener('click', setZero);

  // Elementos de controle não podem ser selecionados, arrastados nem abrir
  // menu; os campos de texto continuam editáveis
  document.querySelectorAll('.no-select').forEach((el) => {
    ['selectstart', 'contextmenu', 'dragstart'].forEach((ev) =>
      el.addEventListener(ev, (e) => {
        if (!e.target.closest?.('input, select')) e.preventDefault();
      }));
  });

  // Segurança: o jog nunca continua sem o operador olhando para a página
  window.addEventListener('blur', stopJog);
  document.addEventListener('visibilitychange', () => { if (document.hidden) stopJog(); });
  window.addEventListener('pointerup', stopJog);

  onConnect((ros) => {
    velPub = topic(ros, CONFIG.topics.cmdVelWeb);
    posPub = topic(ros, CONFIG.topics.cmdPosWeb);
    setZeroSrv = service(ros, CONFIG.services.setZero);
    setGroupEnabled('ros', true);
  });

  onDisconnect(() => {
    stopJog();
    velPub = posPub = setZeroSrv = null;
    setGroupEnabled('ros', false);
  });
}

// ---------------- Velocidade ----------------
function publishVelocity(panDegS, tiltDegS) {
  if (!velPub || !isConnected()) return;
  velPub.publish(new ROSLIB.Message({
    linear: { x: 0, y: 0, z: 0 },
    angular: { x: 0, y: tiltDegS * DEG_TO_RAD, z: panDegS * DEG_TO_RAD },
  }));
}

function stopAll() {
  stopJog();
  publishVelocity(0, 0);
  toast('Parado');
}

// ---------------- Jog ----------------
function initSliders() {
  const { minDegS, maxDegS, panDefaultDegS, tiltDefaultDegS } = CONFIG.jog;
  const defaults = { pan: panDefaultDegS, tilt: tiltDefaultDegS };
  for (const axis of ['pan', 'tilt']) {
    const input = document.getElementById(`speed-${axis}`);
    const label = document.getElementById(`speed-${axis}-lbl`);
    input.min = minDegS;
    input.max = maxDegS;
    input.value = defaults[axis];
    label.textContent = input.value;
    input.addEventListener('input', () => { label.textContent = input.value; });
    sliders[axis] = input;
  }
}

function initJogButtons() {
  document.querySelectorAll('.jog-btn[data-axis]').forEach((btn) => {
    btn.addEventListener('pointerdown', (e) => {
      if (e.button !== 0 || btn.disabled) return;
      e.preventDefault();
      // No toque o navegador captura o ponteiro no alvo; sem liberar, o
      // pointerleave só chegaria ao soltar o dedo
      if (btn.hasPointerCapture(e.pointerId)) btn.releasePointerCapture(e.pointerId);
      startJog(btn);
    });
    btn.addEventListener('pointerup', stopJog);
    btn.addEventListener('pointerleave', stopJog);
    btn.addEventListener('pointercancel', stopJog);
  });
}

function jogVelocity(btn) {
  const axis = btn.dataset.axis;
  const speed = parseFloat(sliders[axis].value) * parseFloat(btn.dataset.dir);
  return axis === 'pan' ? [speed, 0] : [0, speed];
}

function startJog(btn) {
  if (!isConnected()) return;
  if (activeBtn) stopJog();
  activeBtn = btn;
  btn.classList.add('pressed');
  publishVelocity(...jogVelocity(btn));
  jogTimer = setInterval(() => publishVelocity(...jogVelocity(btn)), CONFIG.jogRepeatMs);
}

function stopJog() {
  if (!activeBtn) return;
  clearInterval(jogTimer);
  jogTimer = null;
  activeBtn.classList.remove('pressed');
  activeBtn = null;
  publishVelocity(0, 0);
}

// ---------------- Posição ----------------
function initPosition() {
  for (const axis of ['pan', 'tilt']) {
    const input = document.getElementById(`pos-${axis}`);
    [input.min, input.max] = CONFIG.limitsDeg[axis];
  }
  document.getElementById('pos-form').addEventListener('submit', (e) => {
    e.preventDefault();
    sendPosition();
  });
}

function readPosition(axis) {
  const input = document.getElementById(`pos-${axis}`);
  const [min, max] = CONFIG.limitsDeg[axis];
  const value = Math.min(max, Math.max(min, parseFloat(input.value) || 0));
  input.value = value;
  return value;
}

function sendPosition() {
  if (!posPub || !isConnected()) return;
  stopJog();
  const pan = readPosition('pan');
  const tilt = readPosition('tilt');
  posPub.publish(new ROSLIB.Message({
    name: ['pan_joint', 'tilt_joint'],
    position: [pan * DEG_TO_RAD, tilt * DEG_TO_RAD],
    velocity: [],
    effort: [],
  }));
  toast(`Movendo para pan ${pan}° / tilt ${tilt}°`);
}

// ---------------- Zero ----------------
async function setZero() {
  if (!setZeroSrv) return;
  const ok = window.confirm(
    'Definir a posição atual como zero dos dois eixos?\n' +
    'Os limites de ângulo são relativos ao zero: só confirme com o mecanismo no centro mecânico.');
  if (!ok) return;
  try {
    const res = await callService(setZeroSrv);
    toast(res.message || (res.success ? 'Eixos zerados' : 'Falha ao zerar'), res.success ? 'good' : 'bad');
  } catch (err) {
    toast(`Falha ao zerar: ${err}`, 'bad');
  }
}
