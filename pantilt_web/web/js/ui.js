// Utilitários de interface compartilhados pelos painéis.

let toastTimer = null;

/** Mensagem temporária no rodapé. kind: '', 'good' ou 'bad'. */
export function toast(text, kind = '') {
  const el = document.getElementById('toast');
  el.textContent = text;
  el.className = 'toast show ' + kind;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => el.classList.remove('show'), 2200);
}

/** Habilita ou desabilita todos os controles marcados com data-requires="<grupo>". */
export function setGroupEnabled(group, enabled) {
  document.querySelectorAll(`[data-requires~="${group}"]`).forEach((el) => {
    el.disabled = !enabled;
  });
}

export function formatDeg(value) {
  return (Math.abs(value) < 0.05 ? 0 : value).toFixed(1);
}
