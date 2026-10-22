"""
Figuras dos ensaios (docs/architecture.md, seção 4.11 "Análise"): uma por
gravação e as comparativas entre rótulos, em PNG e PDF.

Largura de 16 cm (coluna de texto do A4), textos em português e vírgula
decimal nos eixos e nos valores (padrão ABNT). Backend Agg: não precisa de
display.
"""

import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.ticker import FuncFormatter  # noqa: E402

from pantilt_experiment.analysis import MODE_TRACK, rows, visible  # noqa: E402

WIDTH_IN = 16.0 / 2.54
DPI = 200
FORMATS = ('png', 'pdf')

STYLE = {
    'font.size': 8,
    'axes.titlesize': 8,
    'axes.labelsize': 8,
    'legend.fontsize': 7,
    'xtick.labelsize': 7,
    'ytick.labelsize': 7,
    'axes.grid': True,
    'grid.alpha': 0.3,
    'lines.linewidth': 1.0,
    'pdf.fonttype': 42,       # fontes embutidas como TrueType (texto editável no PDF)
}

COLOR_PAN = '#5d04d9'
COLOR_TILT = '#d97706'
COLOR_NORM = '#6b7280'
MARKERS = {
    'goal': ('#9ca3af', '--', 'início do goal'),
    't0': ('#0f6b3e', ':', 'primeira detecção'),
    'hold': ('#155a9e', '-.', 'início da permanência'),
    'centered': ('#b71c1c', '-', 'centralizado'),
}


# ---------------- Vírgula decimal ----------------
def number(value: float, digits: int = 2) -> str:
    """Número com vírgula decimal (NaN vira travessão)."""
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return '—'
    return f'{value:.{digits}f}'.replace('.', ',').replace('-', '−')


def _tick(value, _pos=None) -> str:
    text = f'{value:.6g}'
    if text in ('-0', '0'):
        return '0'
    return text.replace('.', ',').replace('-', '−')


COMMA = FuncFormatter(_tick)


def use_comma(ax, x: bool = True, y: bool = True):
    if x:
        ax.xaxis.set_major_formatter(COMMA)
    if y:
        ax.yaxis.set_major_formatter(COMMA)


def _save(fig, base: str) -> list:
    os.makedirs(os.path.dirname(base), exist_ok=True)
    paths = []
    for ext in FORMATS:
        path = f'{base}.{ext}'
        fig.savefig(path, dpi=DPI, bbox_inches='tight')
        paths.append(path)
    plt.close(fig)
    return paths


# ---------------- Por gravação ----------------
def _vlines(ax, a):
    """Marca os instantes do goal principal (e o início de cada goal)."""
    color, style, _ = MARKERS['goal']
    for t in a.goal_starts:
        ax.axvline(t, color=color, linestyle=style, linewidth=0.8)
    for key, t in (('t0', a.t0), ('hold', a.hold_start), ('centered', a.t_centered)):
        if not math.isnan(t):
            color, style, _ = MARKERS[key]
            ax.axvline(t, color=color, linestyle=style, linewidth=0.9)


def plot_recording(a, base: str) -> list:
    """
    Figura de uma gravação em base.png e base.pdf: erro (px), θ (°), comando
    (°/s) e juntas (°) no tempo da gravação. Sem servo.csv, não gera nada.
    """
    servo = a.rec.servo
    if servo is None or rows(servo) == 0:
        return []
    vis = visible(servo)
    joints = a.rec.joints
    r = a.row

    with plt.rc_context(STYLE):
        fig, axes = plt.subplots(4, 1, sharex=True, figsize=(WIDTH_IN, 0.95 * WIDTH_IN))
        ax_e, ax_th, ax_u, ax_j = axes

        ax_e.plot(vis['t'], vis['error_x'], '.-', ms=2, color=COLOR_PAN, label='erro x')
        ax_e.plot(vis['t'], vis['error_y'], '.-', ms=2, color=COLOR_TILT, label='erro y')
        ax_e.plot(vis['t'], vis['error_px'], color=COLOR_NORM, linewidth=0.8, label='|erro|')
        if not math.isnan(a.tolerance_px):
            ax_e.axhline(a.tolerance_px, color=COLOR_NORM, linestyle=':', linewidth=0.8,
                         label=f'tolerância ({number(a.tolerance_px, 0)} px)')
        ax_e.set_ylabel('erro (px)')

        ax_th.plot(vis['t'], np.degrees(vis['theta_pan']), color=COLOR_PAN, label='θ pan')
        ax_th.plot(vis['t'], np.degrees(vis['theta_tilt']), color=COLOR_TILT, label='θ tilt')
        ax_th.axhline(0.0, color='black', linewidth=0.5)
        ax_th.set_ylabel('θ (°)')

        for axis, color in (('pan', COLOR_PAN), ('tilt', COLOR_TILT)):
            cmd = np.degrees(vis[f'cmd_{axis}'])
            ax_u.plot(vis['t'], cmd, drawstyle='steps-post', color=color, label=f'comando {axis}')
            sat = vis[f'saturated_{axis}'] == 1
            if np.any(sat):
                ax_u.plot(vis['t'][sat], cmd[sat], 'x', ms=3, color=color,
                          label=f'saturado ({axis})')
        ax_u.set_ylabel('comando (°/s)')

        if joints is not None and rows(joints):
            ax_j.plot(joints['t'], np.degrees(joints['pan_pos']), color=COLOR_PAN, label='pan')
            ax_j.plot(joints['t'], np.degrees(joints['tilt_pos']), color=COLOR_TILT,
                      label='tilt')
        ax_j.set_ylabel('junta (°)')
        ax_j.set_xlabel('tempo desde o início da gravação (s)')

        for ax in axes:
            _vlines(ax, a)
            use_comma(ax)
            ax.legend(loc='best', ncol=2, framealpha=0.85, edgecolor='none')

        fig.tight_layout()
        # Acima do primeiro painel: a legenda das marcas verticais (uma vez) e
        # o título; o bbox_inches='tight' do _save corta o espaço que sobrar
        top = ax_e.get_position().y1
        handles = [plt.Line2D([], [], color=c, linestyle=s, linewidth=0.9, label=lbl)
                   for c, s, lbl in MARKERS.values()]
        fig.legend(handles=handles, loc='lower center', ncol=4, frameon=False,
                   bbox_to_anchor=(0.5, top))
        rep = r['repeticao']
        rep = f'#{int(rep):03d}' if not (isinstance(rep, float) and math.isnan(rep)) else ''
        fig.suptitle(
            f'{r["rotulo"]} {rep} · {r["equipamento"] or "—"} · {r["modo"]} · {r["desfecho"]}\n'
            f't_c = {number(r["t_c_s"])} s · e_r = {number(r["e_r_px"], 1)} px · '
            f'IAE = {number(r["iae_px_s"], 1)} px·s · '
            f'M_p pan/tilt = {number(r["mp_pan_deg"])}°/{number(r["mp_tilt_deg"])}°',
            y=top + 0.035, va='bottom')
        return _save(fig, base)


# ---------------- Comparativas ----------------
def _by_label(analyses, key):
    labels = sorted({a.row['rotulo'] for a in analyses})
    data = []
    for label in labels:
        values = np.array([key(a) for a in analyses if a.row['rotulo'] == label], dtype=float)
        data.append(values[~np.isnan(values)])
    return labels, data


def _mp_max(a) -> float:
    values = [a.row['mp_pan_deg'], a.row['mp_tilt_deg']]
    values = [v for v in values if not math.isnan(v)]
    return max(values) if values else math.nan


COMPARE_METRICS = (
    ('t_c (s)', lambda a: a.row['t_c_s']),
    ('e_r (px)', lambda a: a.row['e_r_px']),
    ('IAE (px·s)', lambda a: a.row['iae_px_s']),
    ('M_p, maior eixo (°)', _mp_max),
    ('e_d (px, TRACK)', lambda a: a.row['e_d_px']),
)


def _boxplot(ax, labels, data, title):
    positions = [i + 1 for i, d in enumerate(data) if len(d)]
    filled = [d for d in data if len(d)]
    if filled:
        ax.boxplot(filled, positions=positions, widths=0.5, showfliers=False,
                   medianprops={'color': COLOR_PAN})
        rng = np.random.default_rng(0)
        for p, d in zip(positions, filled):
            ax.plot(p + rng.uniform(-0.12, 0.12, len(d)), d, '.', ms=3, color=COLOR_NORM)
    else:
        ax.text(0.5, 0.5, 'sem dados', ha='center', va='center', transform=ax.transAxes,
                color=COLOR_NORM)
    ax.set_xticks(range(1, len(labels) + 1))
    ax.set_xticklabels(labels, rotation=30 if len(labels) > 3 else 0, ha='right'
                       if len(labels) > 3 else 'center')
    ax.set_xlim(0.4, len(labels) + 0.6)
    ax.set_title(title)
    use_comma(ax, x=False)


def plot_comparisons(analyses, out_dir: str) -> list:
    """comparativo_metricas e comparativo_respostas em out_dir (gravações não interrompidas)."""
    analyses = [a for a in analyses if a.row['desfecho'] != 'interrompido']
    if not analyses:
        return []
    paths = []
    with plt.rc_context(STYLE):
        fig, axes = plt.subplots(2, 3, figsize=(WIDTH_IN, 0.62 * WIDTH_IN))
        for ax, (title, key) in zip(axes.flat, COMPARE_METRICS):
            labels, data = _by_label(analyses, key)
            _boxplot(ax, labels, data, title)
        ax = axes.flat[-1]
        labels, data = _by_label(analyses, lambda a: float(a.row['sucesso']))
        rates = [100.0 * np.mean(d) if len(d) else 0.0 for d in data]
        x = np.arange(1, len(labels) + 1)
        ax.bar(x, rates, width=0.5, color=COLOR_PAN)
        for xi, rate, d in zip(x, rates, data):
            ax.text(xi, rate + 2, f'{number(rate, 0)}%\n(n={len(d)})', ha='center', va='bottom',
                    fontsize=6)
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=30 if len(labels) > 3 else 0,
                           ha='right' if len(labels) > 3 else 'center')
        ax.set_ylim(0, 125)
        ax.set_yticks([0, 25, 50, 75, 100])
        ax.set_title('sucesso (%)')
        fig.tight_layout()
        paths += _save(fig, os.path.join(out_dir, 'comparativo_metricas'))

        labels = sorted({a.row['rotulo'] for a in analyses})
        ncols = min(3, len(labels))
        nrows = math.ceil(len(labels) / ncols)
        fig, axes = plt.subplots(nrows, ncols, sharex=True, sharey=True, squeeze=False,
                                 figsize=(WIDTH_IN, 0.36 * WIDTH_IN * nrows + 0.4))
        for ax, label in zip(axes.flat, labels):
            track = False
            for a in analyses:
                if a.row['rotulo'] != label or math.isnan(a.t0):
                    continue
                vis = visible(a.segment)
                is_track = a.row['modo'] == MODE_TRACK
                track = track or is_track
                ax.plot(vis['t'] - a.t0, vis['error_px'], linewidth=0.8, alpha=0.7,
                        linestyle='--' if is_track else '-')
            if track:
                ax.legend(handles=[plt.Line2D([], [], color=COLOR_NORM, label='CENTER'),
                                   plt.Line2D([], [], color=COLOR_NORM, linestyle='--',
                                              label='TRACK')],
                          loc='upper right', framealpha=0.85, edgecolor='none')
            ax.set_title(label)
            use_comma(ax)
        for ax in axes.flat[len(labels):]:
            ax.set_visible(False)
        for ax in axes[-1]:
            ax.set_xlabel('tempo desde a 1ª detecção (s)')
        for ax in axes[:, 0]:
            ax.set_ylabel('|erro| (px)')
        fig.tight_layout()
        paths += _save(fig, os.path.join(out_dir, 'comparativo_respostas'))
    return paths
