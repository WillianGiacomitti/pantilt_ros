// Painel de coleta de dataset: inicia e encerra as gravações do capture_node
// pelos services /capture/start e /capture/stop e mostra o /capture/status
// (architecture.md §4.8 e §4.9).
//
// O capture_node não tem service de consulta: o painel fica indisponível até
// chegar o primeiro /capture/status (transient_local) e volta a ficar se o
// status, publicado a 2 Hz, sumir por CONFIG.capture.statusStaleMs.
//
// O "Parar tudo" e o STOP do jog não passam por aqui: só param o movimento, e a
// varredura volta depois da janela do operador. Para encerrar a varredura, use
// o Parar deste painel.

import { CONFIG } from './config.js';
import { onConnect, onDisconnect, service, callService, subscribeLatched } from './ros.js';
import { toast } from './ui.js';

// Tempo em que uma recusa ou falha fica na linha de mensagem do painel
const NOTICE_MS = 8000;

let srv = {};
let nodeReady = false;
let lastStatus = null;
let lastStatusAt = 0;
let busy = false;          // chamada de service em andamento
let notice = null;         // { text, until }: recusa ou falha a destacar

const el = (id) => document.getElementById(id);

export function init() {
  el('cap-form').addEventListener('submit', (e) => {
    e.preventDefault();
    start();
  });
  el('btn-cap-stop').addEventListener('click', stop);
  el('cap-scan').addEventListener('change', updateButtons);
  el('cap-speed').max = CONFIG.capture.maxSpeedDegS;

  onConnect((ros) => {
    srv = {
      start: service(ros, CONFIG.services.captureStart),
      stop: service(ros, CONFIG.services.captureStop),
    };
    busy = false;      // uma chamada pendente da conexão anterior nunca responde
    subscribeLatched(ros, CONFIG.topics.captureStatus, onStatus);
    setReady(false, 'Aguardando o capture_node…');
  });

  onDisconnect(() => {
    srv = {};
    setReady(false, 'Sem conexão com o rosbridge');
  });

  // Status sumiu: o capture_node saiu do ar ou travou
  setInterval(() => {
    if (nodeReady && performance.now() - lastStatusAt > CONFIG.capture.statusStaleMs) {
      setReady(false, 'capture_node indisponível (sem /capture/status)');
    }
    if (notice && performance.now() > notice.until) {
      notice = null;
      render(lastStatus);
    }
  }, 500);

  setReady(false, 'Conectando…');
}

function setReady(ready, note) {
  nodeReady = ready;
  el('cap-note').textContent = note;
  el('cap-note').classList.toggle('warn', !ready);
  if (!ready) {
    lastStatus = null;
    render(null);
  }
  updateButtons();
}

function updateButtons() {
  const recording = !!(lastStatus && lastStatus.recording);
  const editable = nodeReady && !recording && !busy;
  el('cap-session').disabled = !editable;
  el('cap-scan').disabled = !editable;
  el('cap-speed').disabled = !editable || !el('cap-scan').checked;
  el('btn-cap-start').disabled = !editable;
  el('btn-cap-stop').disabled = !(nodeReady && recording && !busy);
}

// ---------------- Iniciar / parar ----------------
async function start() {
  if (!srv.start || busy) return;
  const session = el('cap-session').value.trim();
  if (!session) {
    toast('Informe o nome da sessão', 'bad');
    return;
  }
  const scan = el('cap-scan').checked;
  const speed = scan ? Number(el('cap-speed').value) || 0 : 0;

  busy = true;
  updateButtons();
  try {
    const res = await callService(srv.start, { session, scan, speed_deg_s: speed });
    const text = res.message || (res.accepted ? 'Gravação iniciada' : 'Gravação recusada');
    toast(text, res.accepted ? 'good' : 'bad');
    setNotice(res.accepted ? null : `Recusado: ${text}`);
  } catch (err) {
    toast(`Falha ao iniciar a gravação: ${err}`, 'bad');
    setNotice(`Falha ao iniciar a gravação: ${err}`);
  } finally {
    busy = false;
    updateButtons();
  }
}

async function stop() {
  if (!srv.stop || busy) return;
  busy = true;
  updateButtons();
  try {
    const res = await callService(srv.stop);
    toast(res.message || (res.success ? 'Gravação encerrada' : 'Falha ao encerrar'),
          res.success ? 'good' : 'bad');
    if (!res.success) setNotice(res.message);
  } catch (err) {
    toast(`Falha ao encerrar a gravação: ${err}`, 'bad');
    setNotice(`Falha ao encerrar a gravação: ${err}`);
  } finally {
    busy = false;
    updateButtons();
  }
}

function setNotice(text) {
  notice = text ? { text, until: performance.now() + NOTICE_MS } : null;
  render(lastStatus);
}

// ---------------- Estado ----------------
function onStatus(msg) {
  lastStatusAt = performance.now();
  if (!nodeReady) setReady(true, '');
  lastStatus = msg;
  render(msg);
  updateButtons();
}

function render(msg) {
  const recording = !!(msg && msg.recording);
  const scanning = recording && msg.scanning;

  let state = '';
  let label = '—';
  if (msg) {
    state = scanning ? 'scanning' : recording ? 'recording' : 'idle';
    label = scanning ? 'Gravando + varredura' : recording ? 'Gravando' : 'Parado';
  }
  el('cap-state').textContent = label;
  el('cap-state').dataset.state = state;

  el('cap-time').textContent = recording ? formatTime(msg.elapsed_s) : '—';

  const frames = el('cap-frames');
  if (msg && (recording || msg.frames)) {
    frames.textContent = msg.dropped ? `${msg.frames} · ${msg.dropped} perd.` : `${msg.frames}`;
    frames.title = msg.dropped ? `${msg.dropped} quadros descartados por fila cheia` : '';
    frames.classList.toggle('warn', msg.dropped > 0);
  } else {
    frames.textContent = '—';
    frames.title = '';
    frames.classList.remove('warn');
  }

  const file = el('cap-file');
  file.textContent = msg && msg.file ? msg.file.split('/').pop() : '—';
  file.title = msg && msg.file ? msg.file : '';

  const message = el('cap-message');
  message.textContent = notice ? notice.text : (msg ? msg.message : '');
  message.classList.toggle('bad', !!notice);

  el('rec-chip').hidden = !recording;
  el('rec-time').textContent = recording ? formatTime(msg.elapsed_s) : '';
  el('tab-rec').hidden = !recording;
}

/** Segundos em mm:ss (ou h:mm:ss). */
function formatTime(seconds) {
  const total = Math.max(0, Math.floor(seconds));
  const h = Math.floor(total / 3600);
  const m = Math.floor((total % 3600) / 60);
  const s = total % 60;
  const mm = String(m).padStart(2, '0');
  const ss = String(s).padStart(2, '0');
  return h ? `${h}:${mm}:${ss}` : `${mm}:${ss}`;
}
