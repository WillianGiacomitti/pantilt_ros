// Diagnóstico: conexão com o rosbridge, fonte de controle ativa
// (/ptu/control_source, do command_mux) e eventos da ESP32 (/ptu/errors,
// do serial_bridge_node).

import { CONFIG } from './config.js';
import { onConnect, onDisconnect, subscribeLatched } from './ros.js';

const MAX_ENTRIES = 50;

const SOURCE_LABELS = {
  web: 'Operador',
  auto: 'Automático',
  none: 'Nenhuma',
};

let errorCount = 0;

const el = (id) => document.getElementById(id);

export function init() {
  onConnect((ros) => {
    setConnection(true);
    // /ptu/errors é transient_local com histórico: ao reassinar, as mesmas
    // mensagens chegam de novo. A lista é refeita a cada conexão.
    clearLog();
    subscribeLatched(ros, CONFIG.topics.errors, onError);
    subscribeLatched(ros, CONFIG.topics.controlSource, onControlSource);
  });

  onDisconnect(() => {
    setConnection(false);
    onControlSource(null);
  });

  setConnection(false);
  onControlSource(null);
}

function setConnection(online) {
  el('conn-pill').classList.toggle('on', online);
  el('conn-text').textContent = online ? 'Online' : 'Offline';
  document.body.classList.toggle('offline', !online);
}

function onControlSource(msg) {
  const src = msg ? msg.data : '';
  el('source-pill').dataset.source = src;
  el('source-text').textContent = msg ? (SOURCE_LABELS[src] || src) : '—';
}

function clearLog() {
  errorCount = 0;
  el('err-count').textContent = '';
  el('log-list').replaceChildren(emptyEntry());
}

function emptyEntry() {
  const div = document.createElement('div');
  div.className = 'log-empty';
  div.textContent = 'Nenhum evento ainda.';
  return div;
}

function onError(msg) {
  const text = msg.data || '';
  const list = el('log-list');
  const empty = list.querySelector('.log-empty');
  if (empty) empty.remove();

  const isError = text.includes('ERRO');
  const isBoot = text.includes('BOOT') || text.includes('Boot');

  // O serial_bridge_node prefixa o horário: "[HH:MM:SS] texto"
  const match = text.match(/^\[(\d{2}:\d{2}:\d{2})\]\s*(.*)$/s);

  const entry = document.createElement('div');
  entry.className = 'log-entry ' + (isError ? 'is-error' : isBoot ? 'is-boot' : 'is-info');
  const time = document.createElement('span');
  time.className = 'log-time';
  time.textContent = match ? match[1] : new Date().toLocaleTimeString('pt-BR');
  const body = document.createElement('span');
  body.textContent = match ? match[2] : text;
  entry.append(time, body);
  list.prepend(entry);

  while (list.children.length > MAX_ENTRIES) list.lastChild.remove();

  if (isError) {
    errorCount++;
    el('err-count').textContent = `${errorCount} erro(s)`;
  }
}
