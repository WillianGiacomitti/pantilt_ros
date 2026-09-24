// Conexão com o rosbridge.
//
// Cada conexão usa um ROSLIB.Ros novo, e os módulos recriam seus tópicos e
// services no onConnect. Assim nenhum comando fica enfileirado pelo roslib
// durante a queda para ser enviado depois, fora de hora, ao reconectar.

import { CONFIG } from './config.js';

const connectHooks = [];
const disconnectHooks = [];
let ros = null;
let connected = false;

/** Registra fn(ros), chamada a cada conexão estabelecida. */
export function onConnect(fn) {
  connectHooks.push(fn);
  if (connected) fn(ros);
}

/** Registra fn(), chamada quando a conexão cai. */
export function onDisconnect(fn) {
  disconnectHooks.push(fn);
}

export function isConnected() {
  return connected && ros !== null && ros.isConnected;
}

export function start() {
  connect();
}

function connect() {
  const current = new ROSLIB.Ros({ url: CONFIG.rosbridgeUrl });
  ros = current;

  current.on('connection', () => {
    if (current !== ros) return;
    connected = true;
    connectHooks.forEach((fn) => fn(current));
  });

  // Sem listener, o EventEmitter do roslib lança exceção no 'error'. Ele sempre
  // vem acompanhado de 'close', onde fica a reconexão.
  current.on('error', () => {});

  current.on('close', () => {
    if (current !== ros) return;
    const wasConnected = connected;
    connected = false;
    if (wasConnected) disconnectHooks.forEach((fn) => fn());
    setTimeout(connect, CONFIG.reconnectMs);
  });
}

/**
 * Cria um ROSLIB.Topic a partir de uma entrada de CONFIG.topics.
 *
 * Não usar reconnect_on_close: false: no roslib 1.4.1 essa opção quebra o
 * subscribe. O que o roslib enfileira no Ros antigo após a queda nunca é
 * enviado, porque aquele objeto não reconecta.
 */
export function topic(r, def, options = {}) {
  return new ROSLIB.Topic({ ros: r, name: def.name, messageType: def.type, ...options });
}

/** Cria um ROSLIB.Service a partir de uma entrada de CONFIG.services. */
export function service(r, def) {
  return new ROSLIB.Service({ ros: r, name: def.name, serviceType: def.type });
}

/** Chama um service e devolve uma Promise (rejeita com a mensagem de erro do rosbridge). */
export function callService(srv, request = {}) {
  return new Promise((resolve, reject) => {
    srv.callService(new ROSLIB.ServiceRequest(request), resolve, (err) => reject(String(err)));
  });
}

/**
 * Assina um tópico transient_local.
 *
 * O rosbridge escolhe o QoS da assinatura no momento em que ela é criada: se
 * ainda não existe publicador transient_local, assina como volatile e perde o
 * último estado publicado (ex.: /ptu/control_source só é publicado na troca).
 * Por isso, até chegar a primeira mensagem, a assinatura é refeita com
 * intervalo crescente (nextRetryMs).
 */
export function subscribeLatched(r, def, callback) {
  const t = topic(r, def);
  let received = false;
  let delay = 0;
  const handler = (msg) => {
    received = true;
    callback(msg);
  };
  t.subscribe(handler);

  const retry = () => {
    if (received || !r.isConnected) return;
    t.unsubscribe(handler);
    t.subscribe(handler);
    delay = nextRetryMs(delay);
    setTimeout(retry, delay);
  };
  delay = nextRetryMs(delay);
  setTimeout(retry, delay);

  return t;
}

/** Próximo intervalo de nova tentativa: CONFIG.retryMinMs, dobrando até CONFIG.retryMaxMs. */
export function nextRetryMs(previous) {
  return previous ? Math.min(previous * 2, CONFIG.retryMaxMs) : CONFIG.retryMinMs;
}
