// Painel de inspeção: seleção do equipamento, iniciar/parar e estado do
// inspection_manager.
//
// A web não usa actions (limitação do rosbridge/roslibjs no Humble): usa os
// services /inspection/* e o tópico /inspection/status (architecture.md §4.8).
// Enquanto o inspection_manager não responde, o painel fica indisponível e a
// lista de equipamentos é pedida de novo com intervalo crescente. O primeiro
// /inspection/status recebido indica que o gerenciador subiu e antecipa o pedido.

import { CONFIG, MODE_CENTER, MODE_TRACK } from './config.js';
import { onConnect, onDisconnect, service, callService, subscribeLatched, nextRetryMs } from './ros.js';
import { toast, setGroupEnabled } from './ui.js';

const STATE_LABELS = {
  IDLE: 'Ocioso',
  SEARCHING: 'Varredura',
  CENTERING: 'Centralizando',
  TRACKING: 'Rastreando',
};

let srv = {};
let retryTimer = null;
let retryDelay = 0;
let loading = false;
let managerReady = false;
let lastStatus = null;

const el = (id) => document.getElementById(id);

export function init() {
  el('insp-form').addEventListener('submit', (e) => {
    e.preventDefault();
    start();
  });
  el('btn-insp-stop').addEventListener('click', stop);

  onConnect((ros) => {
    srv = {
      list: service(ros, CONFIG.services.listEquipment),
      start: service(ros, CONFIG.services.start),
      stop: service(ros, CONFIG.services.stop),
    };
    subscribeLatched(ros, CONFIG.topics.status, onStatus);
    retryDelay = 0;
    loading = false;   // uma chamada pendente da conexão anterior nunca responde
    loadEquipment();
  });

  onDisconnect(() => {
    clearTimeout(retryTimer);
    srv = {};
    setManagerReady(false, 'Sem conexão com o rosbridge');
    onStatus(null);
  });

  setManagerReady(false, 'Conectando…');
  onStatus(null);
}

// ---------------- Equipamentos ----------------
async function loadEquipment() {
  const listSrv = srv.list;
  if (!listSrv || loading) return;
  clearTimeout(retryTimer);
  loading = true;
  try {
    const res = await callService(listSrv);
    if (listSrv !== srv.list) return;      // conexão trocou durante a chamada
    fillEquipment(res.keys, res.labels);
    setManagerReady(true, res.keys.length ? '' : 'Nenhum equipamento disponível no modelo');
  } catch (err) {
    if (listSrv !== srv.list) return;
    setManagerReady(false, 'Gerenciador indisponível (inspection_manager não respondeu)');
    retryDelay = nextRetryMs(retryDelay);
    retryTimer = setTimeout(loadEquipment, retryDelay);
  } finally {
    loading = false;
  }
}

function fillEquipment(keys, labels) {
  const select = el('insp-equipment');
  const previous = select.value;
  select.replaceChildren(...keys.map((key, i) => {
    const opt = document.createElement('option');
    opt.value = key;
    opt.textContent = labels[i] || key;
    return opt;
  }));
  if (keys.includes(previous)) select.value = previous;
}

function setManagerReady(ready, note) {
  managerReady = ready;
  el('insp-panel').classList.toggle('unavailable', !ready);
  el('insp-note').textContent = note;
  if (!ready) el('insp-equipment').replaceChildren();
  updateButtons();
}

function updateButtons() {
  setGroupEnabled('manager', managerReady);
  const idle = !lastStatus || lastStatus.state === 'IDLE';
  const hasEquipment = el('insp-equipment').options.length > 0;
  el('btn-insp-start').disabled = !(managerReady && idle && hasEquipment);
}

// ---------------- Iniciar / parar ----------------
async function start() {
  if (!srv.start) return;
  const equipment = el('insp-equipment').value;
  const mode = el('insp-mode-track').checked ? MODE_TRACK : MODE_CENTER;
  try {
    const res = await callService(srv.start, { equipment, mode });
    toast(res.message || (res.accepted ? 'Inspeção iniciada' : 'Inspeção recusada'),
          res.accepted ? 'good' : 'bad');
  } catch (err) {
    toast(`Falha ao iniciar: ${err}`, 'bad');
  }
}

async function stop() {
  if (!srv.stop) return;
  try {
    const res = await callService(srv.stop);
    toast(res.message || (res.success ? 'Inspeção parada' : 'Falha ao parar'),
          res.success ? 'good' : 'bad');
  } catch (err) {
    toast(`Falha ao parar: ${err}`, 'bad');
  }
}

// ---------------- Estado ----------------
function onStatus(msg) {
  if (msg && !managerReady && srv.list) loadEquipment();
  lastStatus = msg;
  const state = msg ? msg.state : '';
  const label = msg ? (STATE_LABELS[state] || state) : '—';
  const tracking = state === 'CENTERING' || state === 'TRACKING';

  el('insp-state').textContent = label;
  el('insp-state').dataset.state = state;
  el('insp-current').textContent = msg && msg.equipment ? msg.equipment : '—';
  el('insp-error').textContent = msg && tracking ? `${msg.error_px.toFixed(1)} px` : '—';
  el('insp-message').textContent = msg && msg.message ? msg.message : '';

  const pill = el('state-pill');
  pill.dataset.state = state;
  el('state-text').textContent = label;

  const autonomous = !!(msg && msg.autonomous);
  document.body.classList.toggle('autonomous', autonomous);
  el('auto-banner-detail').textContent =
    autonomous ? `${label}${msg.equipment ? ' · ' + msg.equipment : ''}` : '';

  updateButtons();
}
