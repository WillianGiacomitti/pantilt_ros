// Bloco Ensaio da aba Inspeção: configura o experiment_recorder pelo
// /experiment/configure e mostra o /experiment/status (architecture.md §4.8 e
// §4.11).
//
// Como no painel de coleta, o bloco fica indisponível até chegar o primeiro
// /experiment/status (transient_local) e volta a ficar se o status, publicado a
// 2 Hz, sumir por CONFIG.experiment.statusStaleMs.
//
// Os campos acompanham o status enquanto o operador não os edita: uma
// configuração feita pela CLI também aparece. Com mudanças não aplicadas, o
// Iniciar da inspeção aplica antes (applyIfDirty), para que a gravação não saia
// com o rótulo antigo.

import { CONFIG, IMAGE_MODES } from './config.js';
import { onConnect, onDisconnect, service, callService, subscribeLatched } from './ros.js';
import { toast } from './ui.js';

const LABEL_PLACEHOLDER = 'Rótulo (ex.: degrau_kp0.8)';

// Duração dos avisos do gravador (fim de gravação, gravação não iniciada)
const NOTICE_TOAST_MS = 6000;

let srv = {};
let nodeReady = false;
let note = '';             // motivo da indisponibilidade
let lastStatus = null;
let lastStatusAt = 0;
let lastMessage = null;    // último message do status (aviso só quando muda)
let busy = false;          // chamada de service em andamento
let dirty = false;         // campos editados e ainda não aplicados

const el = (id) => document.getElementById(id);

export function init() {
  el('exp-image').replaceChildren(...IMAGE_MODES.map(({ value, label }) => {
    const opt = document.createElement('option');
    opt.value = value;
    opt.textContent = label;
    return opt;
  }));

  el('exp-form').addEventListener('submit', (e) => {
    e.preventDefault();
    apply();
  });
  el('exp-label').addEventListener('input', markDirty);
  el('exp-notes').addEventListener('input', markDirty);
  el('exp-image').addEventListener('change', markDirty);

  onConnect((ros) => {
    srv = { configure: service(ros, CONFIG.services.experimentConfigure) };
    busy = false;      // uma chamada pendente da conexão anterior nunca responde
    subscribeLatched(ros, CONFIG.topics.experimentStatus, onStatus);
    setReady(false, 'Aguardando o experiment_recorder…');
  });

  onDisconnect(() => {
    srv = {};
    setReady(false, 'Sem conexão com o rosbridge');
  });

  // Status sumiu: o experiment_recorder saiu do ar ou travou
  setInterval(() => {
    if (nodeReady && performance.now() - lastStatusAt > CONFIG.experiment.statusStaleMs) {
      setReady(false, 'experiment_recorder indisponível (sem /experiment/status)');
    }
  }, 500);

  setReady(false, 'Conectando…');
}

/**
 * Aplica a configuração se houver mudanças não aplicadas. Devolve null se está
 * tudo aplicado (ou se não há gravador para configurar) e o motivo da recusa
 * em caso contrário.
 */
export async function applyIfDirty() {
  if (!dirty || !nodeReady || !srv.configure) return null;
  const { ok, message } = await configure();
  return ok ? null : message;
}

function setReady(ready, reason) {
  nodeReady = ready;
  note = reason;
  if (!ready) lastStatus = null;
  render();
}

function markDirty() {
  dirty = true;
  render();
}

// ---------------- Configuração ----------------
async function apply() {
  if (!srv.configure || busy) return;
  const { ok, message } = await configure();
  toast(message, ok ? 'good' : 'bad');
}

async function configure() {
  busy = true;
  render();
  try {
    const res = await callService(srv.configure, {
      label: el('exp-label').value.trim(),
      notes: el('exp-notes').value.trim(),
      image_mode: el('exp-image').value,
    });
    // O status repete a mesma mensagem: sem um segundo aviso
    lastMessage = res.message;
    if (res.success) dirty = false;
    return {
      ok: res.success,
      message: res.message || (res.success ? 'Ensaio configurado' : 'Configuração recusada'),
    };
  } catch (err) {
    return { ok: false, message: `Falha ao configurar o ensaio: ${err}` };
  } finally {
    busy = false;
    render();
  }
}

// ---------------- Estado ----------------
function onStatus(msg) {
  lastStatusAt = performance.now();
  const first = !nodeReady;
  if (first) setReady(true, '');
  lastStatus = msg;
  if (!dirty) fill(msg);

  // Aviso quando o gravador muda de mensagem parado: fim de uma gravação
  // ("Gravado: ...") ou gravação não iniciada (ex.: pouco espaço em disco)
  const text = msg.message || '';
  if (!first && !msg.recording && text && text !== lastMessage) {
    toast(text, '', NOTICE_TOAST_MS);
  }
  lastMessage = text;
  render();
}

/** Copia a configuração do status para os campos (só os que mudaram, para não mover o cursor). */
function fill(msg) {
  const set = (id, value) => {
    if (el(id).value !== value) el(id).value = value;
  };
  set('exp-label', msg.label);
  set('exp-notes', msg.notes);
  if (IMAGE_MODES.some((m) => m.value === msg.image_mode)) set('exp-image', msg.image_mode);
}

function render() {
  const msg = lastStatus;
  const recording = !!(msg && msg.recording);
  const editable = nodeReady && !recording && !busy;

  ['exp-label', 'exp-notes', 'exp-image', 'btn-exp-apply'].forEach((id) => {
    el(id).disabled = !editable;
  });
  el('btn-exp-apply').classList.toggle('pending', dirty);
  el('btn-exp-apply').title = dirty ? 'Configuração alterada e ainda não aplicada' : '';
  el('exp-label').placeholder = nodeReady ? LABEL_PLACEHOLDER : note;

  const box = el('exp-rec');
  if (!msg) {
    box.textContent = '—';
    box.dataset.state = '';
    box.title = note;
  } else {
    const rep = `#${String(msg.repetition).padStart(3, '0')}`;
    box.textContent = recording ? `● ${rep} ${formatTime(msg.elapsed_s)}` : `próx. ${rep}`;
    box.dataset.state = recording ? 'recording' : 'idle';
    box.title = [msg.message, msg.folder].filter(Boolean).join('\n');
  }

  el('exp-chip').hidden = !recording;
  el('exp-chip-text').textContent = recording
    ? `${msg.label} #${String(msg.repetition).padStart(3, '0')} ${formatTime(msg.elapsed_s)}`
    : '';
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
