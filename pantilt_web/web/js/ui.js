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

/**
 * Abas: botões [role="tab"][data-tab="<nome>"] dentro de container e painéis
 * [data-tab-panel="<nome>"]. Mostra a aba initial e chama onChange(nome) a cada
 * troca, inclusive na primeira.
 */
export function initTabs(container, initial, onChange) {
  const tabs = [...container.querySelectorAll('[role="tab"][data-tab]')];
  const panels = [...container.querySelectorAll('[data-tab-panel]')];
  const select = (name) => {
    tabs.forEach((t) => t.setAttribute('aria-selected', String(t.dataset.tab === name)));
    panels.forEach((p) => { p.hidden = p.dataset.tabPanel !== name; });
    onChange(name);
  };
  tabs.forEach((t) => t.addEventListener('click', () => select(t.dataset.tab)));
  select(tabs.some((t) => t.dataset.tab === initial) ? initial : tabs[0].dataset.tab);
}

export function formatDeg(value) {
  return (Math.abs(value) < 0.05 ? 0 : value).toFixed(1);
}
