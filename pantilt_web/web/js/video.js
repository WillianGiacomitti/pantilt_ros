// Vídeo com as bboxes: stream MJPEG do web_video_server
// (/perception/debug_image, publicado pelo detector_node) e contagem de
// /perception/detections.
//
// Enquanto nenhum quadro chegar, o painel mostra um aviso no lugar do vídeo.
// Se o web_video_server não responder, o stream é pedido de novo a cada
// CONFIG.videoRetryMs.

import { CONFIG } from './config.js';
import { onConnect, topic } from './ros.js';

// Sem /perception/detections por mais que isso, o contador é escondido
const DETECTIONS_STALE_MS = 2000;

let img;
let placeholder;
let retryTimer = null;
let lastDetectionsAt = 0;

export function init() {
  img = document.getElementById('video');
  placeholder = document.getElementById('video-placeholder');
  document.getElementById('video-topic').textContent = CONFIG.video.topic;

  img.addEventListener('load', () => setHasVideo(true));
  img.addEventListener('error', () => {
    setHasVideo(false, 'Servidor de vídeo indisponível');
    clearTimeout(retryTimer);
    retryTimer = setTimeout(loadStream, CONFIG.videoRetryMs);
  });

  // Verifica se já chegou algum quadro (o load do MJPEG não é garantido em todo navegador)
  setInterval(() => {
    if (img.naturalWidth > 0) setHasVideo(true);
  }, 1000);

  // Ao (re)conectar ao rosbridge, o launch pode ter sido reiniciado: pede o stream de novo
  onConnect((ros) => {
    loadStream();
    topic(ros, CONFIG.topics.detections, { throttle_rate: 200 }).subscribe(onDetections);
  });

  const chip = document.getElementById('det-chip');
  setInterval(() => {
    chip.hidden = performance.now() - lastDetectionsAt > DETECTIONS_STALE_MS;
  }, 500);

  loadStream();
}

function loadStream() {
  clearTimeout(retryTimer);
  const { topic: t, type } = CONFIG.video;
  setHasVideo(false, 'Aguardando imagens');
  img.src = `${CONFIG.videoBaseUrl}/stream?topic=${encodeURIComponent(t)}&type=${type}&_=${Date.now()}`;
}

function setHasVideo(hasVideo, reason = '') {
  img.hidden = !hasVideo;
  placeholder.hidden = hasVideo;
  if (reason) document.getElementById('video-reason').textContent = reason;
}

function onDetections(msg) {
  lastDetectionsAt = performance.now();
  const n = msg.detections.length;
  document.getElementById('det-count').textContent = `${n} detecç${n === 1 ? 'ão' : 'ões'}`;
}
