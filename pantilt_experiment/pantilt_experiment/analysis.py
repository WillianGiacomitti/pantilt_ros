"""
Métricas dos ensaios a partir das pastas do experiment_recorder
(docs/architecture.md, seção 4.11 "Análise", e seção 10). Só numpy: as
figuras ficam em plots.py.

Uma linha por gravação. t_c, e_r, M_p, IAE e e_d vêm do goal principal, o da
primeira convergência (o primeiro com centered=true no center_feedback.csv;
sem nenhum, o primeiro goal). Readquisições e sucesso valem para a gravação
inteira. Métrica que não se aplica ou não pôde ser calculada vale NaN.
"""

import csv
import json
import math
import os
from dataclasses import dataclass, field

import numpy as np

# Colunas de texto dos CSVs (as demais são números; booleanos gravados como 1/0)
STRING_COLUMNS = {'equipment', 'class_name', 'controller', 'goal', 'source', 'event', 'detail'}

# Abaixo disso, o M_p em % do erro inicial só mediria ruído
MP_MIN_DEG = 1.0

# Desfechos (session.py)
SUCCESS_OUTCOMES = ('centralizado', 'rastreou')
OUTCOME_INTERRUPTED = 'interrompido'
SOURCE_INSPECTION = 'inspecao'
MODE_TRACK = 'track'

ANALYSIS_DIR = '_analise'

# Colunas do metricas.csv: contexto da gravação e métricas (seção 10)
CONTEXT_COLUMNS = (
    'rotulo', 'repeticao', 'pasta', 'origem', 'equipamento', 'modo', 'desfecho', 'sucesso',
    'inicio', 'duracao_s', 'modo_imagem', 'controller', 'pan_kp', 'pan_ki', 'pan_kd',
    'tilt_kp', 'tilt_ki', 'tilt_kd', 'max_vel_deg_s', 'f_x', 'f_y', 'goals_center',
    'readquisicoes',
)
METRIC_COLUMNS = (
    't_c_s', 'e_r_px', 'e_d_px', 'mp_pan_deg', 'mp_pan_pct', 'mp_tilt_deg', 'mp_tilt_pct',
    'iae_px_s', 't_b_s', 't_i_s', 'saturacao_pct', 'det_fps', 'det_inf_ms', 'det_inf_p95_ms',
    'det_lat_ms', 'det_lat_p95_ms',
)
METRICS_CSV_COLUMNS = CONTEXT_COLUMNS + METRIC_COLUMNS

# Métricas resumidas por rótulo (além de gravacoes e interrompidas)
SUMMARY_METRICS = ('sucesso', 'readquisicoes') + METRIC_COLUMNS
SUMMARY_COLUMNS = ('rotulo', 'metrica', 'n', 'media', 'desvio', 'minimo', 'maximo')

NAN = float('nan')


# ---------------- Leitura ----------------
def _float(text: str) -> float:
    try:
        return float(text)
    except ValueError:
        return NAN


def load_csv(path: str):
    """
    Lê um CSV do experiment_recorder como {coluna: numpy.ndarray}; None se o
    arquivo não existe (CSV sem linhas não é criado). Colunas de texto ficam
    como object, as demais como float (vazio -> NaN).
    """
    if not os.path.isfile(path):
        return None
    with open(path, newline='', encoding='utf-8') as f:
        reader = csv.reader(f)
        header = next(reader, None)
        rows = list(reader)
    if header is None:
        return None
    table = {}
    for i, name in enumerate(header):
        values = [r[i] if i < len(r) else '' for r in rows]
        if name in STRING_COLUMNS:
            table[name] = np.array(values, dtype=object)
        else:
            table[name] = np.array([_float(v) for v in values], dtype=float)
    return table


def subset(table, mask):
    """Linhas de table selecionadas por mask (None continua None)."""
    if table is None:
        return None
    return {name: values[mask] for name, values in table.items()}


def rows(table) -> int:
    return 0 if table is None else len(next(iter(table.values()), []))


@dataclass
class Recording:
    """Uma pasta de gravação: resumo.json e os CSVs usados nas métricas."""
    folder: str
    summary: dict
    servo: dict = None
    feedback: dict = None
    events: dict = None
    timing: dict = None
    joints: dict = None


def load_recording(folder: str) -> Recording:
    with open(os.path.join(folder, 'resumo.json'), encoding='utf-8') as f:
        summary = json.load(f)
    csv_path = lambda name: os.path.join(folder, f'{name}.csv')  # noqa: E731
    return Recording(
        folder=folder, summary=summary,
        servo=load_csv(csv_path('servo')),
        feedback=load_csv(csv_path('center_feedback')),
        events=load_csv(csv_path('events')),
        timing=load_csv(csv_path('timing')),
        joints=load_csv(csv_path('joints')))


def find_recordings(root: str, labels=None):
    """
    Pastas de gravação em root/<rótulo>/<gravação>, em ordem. Devolve
    (com_resumo, sem_resumo): as sem resumo.json estão em andamento.
    Ignora as pastas que começam com "_" (ex.: _analise).
    """
    found, pending = [], []
    if not os.path.isdir(root):
        return found, pending
    for label in sorted(os.listdir(root)):
        label_dir = os.path.join(root, label)
        if label.startswith('_') or not os.path.isdir(label_dir):
            continue
        if labels and label not in labels:
            continue
        for name in sorted(os.listdir(label_dir)):
            folder = os.path.join(label_dir, name)
            if not os.path.isdir(folder):
                continue
            if os.path.isfile(os.path.join(folder, 'resumo.json')):
                found.append(folder)
            else:
                pending.append(folder)
    return found, pending


# ---------------- Goals ----------------
def center_goals(summary: dict) -> list:
    """Goals Center do resumo.json em ordem de início."""
    goals = [g for g in summary.get('center') or [] if g.get('t_inicio') is not None]
    return sorted(goals, key=lambda g: g['t_inicio'])


def goal_feedback(feedback, goal: dict):
    """Linhas do center_feedback.csv de um goal (coluna goal = 8 primeiros hex)."""
    if feedback is None:
        return None
    return subset(feedback, feedback['goal'] == goal.get('goal'))


def converged(feedback, goal: dict) -> bool:
    fb = goal_feedback(feedback, goal)
    return fb is not None and bool(np.any(fb['centered'] == 1))


def principal_goal(goals: list, feedback):
    """Goal da primeira convergência; sem nenhuma, o primeiro; None sem goals."""
    for goal in goals:
        if converged(feedback, goal):
            return goal
    return goals[0] if goals else None


def servo_segment(servo, goal: dict):
    """Linhas do servo.csv recebidas durante o goal (t_rx entre o início e o fim)."""
    if servo is None or goal is None:
        return None
    end = goal.get('t_fim')
    end = math.inf if end is None else end
    return subset(servo, (servo['t_rx'] >= goal['t_inicio']) & (servo['t_rx'] <= end))


def visible(segment):
    """Só as linhas com alvo visível."""
    if segment is None:
        return None
    return subset(segment, segment['target_visible'] == 1)


# ---------------- Métricas ----------------
def iae(t, error, t_start: float, t_end: float) -> float:
    """Integral de |erro| (trapézio) entre t_start e t_end; NaN com menos de 2 amostras."""
    t = np.asarray(t, dtype=float)
    error = np.abs(np.asarray(error, dtype=float))
    mask = (t >= t_start) & (t <= t_end)
    t, error = t[mask], error[mask]
    if len(t) < 2:
        return NAN
    order = np.argsort(t, kind='stable')
    t, error = t[order], error[order]
    return float(np.sum((error[1:] + error[:-1]) * np.diff(t)) / 2.0)


def overshoot(theta_rad):
    """
    Sobressinal de um eixo: maior |θ| com sinal oposto ao do θ inicial (depois
    do primeiro cruzamento de zero), em graus, e em % do |θ| inicial. Sem
    cruzamento, 0. O % vale NaN se |θ0| < MP_MIN_DEG. Sem amostras: NaN, NaN.
    """
    theta = np.degrees(np.asarray(theta_rad, dtype=float))
    theta = theta[~np.isnan(theta)]
    if len(theta) == 0:
        return NAN, NAN
    nonzero = np.nonzero(theta)[0]
    if len(nonzero) == 0:
        return 0.0, NAN
    theta0 = theta[nonzero[0]]
    opposite = theta[theta * np.sign(theta0) < 0]
    mp = float(np.max(np.abs(opposite))) if len(opposite) else 0.0
    pct = 100.0 * mp / abs(theta0) if abs(theta0) >= MP_MIN_DEG else NAN
    return mp, pct


def tracking_error(feedback, goal: dict) -> float:
    """e_d: média de error_px do primeiro centered=true ao fim do goal, só com alvo visível."""
    fb = goal_feedback(feedback, goal)
    if fb is None:
        return NAN
    centered = np.nonzero(fb['centered'] == 1)[0]
    if len(centered) == 0:
        return NAN
    after = np.arange(rows(fb)) >= centered[0]
    values = fb['error_px'][after & (fb['target_visible'] == 1)]
    return float(np.mean(values)) if len(values) else NAN


def inspection_start(events) -> float:
    """Início da inspeção: primeiro evento inspecao diferente de IDLE (NaN sem ele)."""
    if events is None:
        return NAN
    mask = (events['source'] == SOURCE_INSPECTION) & (events['event'] != 'IDLE')
    return float(events['t'][mask][0]) if np.any(mask) else NAN


def search_times(events, servo):
    """(t_b, t_i): do início da inspeção à primeira entrada em CENTERING e ao primeiro centered."""
    start = inspection_start(events)
    if math.isnan(start):
        return NAN, NAN
    centering = (events['source'] == SOURCE_INSPECTION) & (events['event'] == 'CENTERING')
    t_b = float(events['t'][centering][0]) - start if np.any(centering) else NAN
    t_i = NAN
    if servo is not None and np.any(servo['centered'] == 1):
        t_i = float(servo['t'][servo['centered'] == 1][0]) - start
    return t_b, t_i


def saturation_pct(servo) -> float:
    """% das linhas visíveis do servo.csv com algum eixo saturado."""
    vis = visible(servo)
    if vis is None or rows(vis) == 0:
        return NAN
    saturated = (vis['saturated_pan'] == 1) | (vis['saturated_tilt'] == 1)
    return float(100.0 * np.mean(saturated))


def detector_stats(timing) -> dict:
    """Taxa (1/intervalo médio entre header.stamp), inferência e atraso: média e p95."""
    out = dict.fromkeys(('det_fps', 'det_inf_ms', 'det_inf_p95_ms', 'det_lat_ms',
                         'det_lat_p95_ms'), NAN)
    if timing is None or rows(timing) == 0:
        return out
    t = np.sort(timing['t'])
    if len(t) >= 2 and np.mean(np.diff(t)) > 0:
        out['det_fps'] = float(1.0 / np.mean(np.diff(t)))
    out['det_inf_ms'] = float(np.mean(timing['inference_ms']))
    out['det_inf_p95_ms'] = float(np.percentile(timing['inference_ms'], 95))
    out['det_lat_ms'] = float(np.mean(timing['latency_ms']))
    out['det_lat_p95_ms'] = float(np.percentile(timing['latency_ms'], 95))
    return out


# ---------------- Gravação ----------------
@dataclass
class Analysis:
    """Métricas de uma gravação e os instantes marcados nas figuras."""
    rec: Recording
    row: dict
    goal: dict = None                 # goal principal
    segment: dict = None              # servo.csv do goal principal
    t0: float = NAN                   # primeira detecção válida do goal principal
    hold_start: float = NAN           # início da janela de permanência (t0 + t_c)
    t_centered: float = NAN           # primeiro centered=true do goal principal
    goal_starts: list = field(default_factory=list)
    tolerance_px: float = NAN         # do inspection_manager (só inspeções)


def _param(params: dict, node: str, name: str):
    values = params.get(node)
    if not isinstance(values, dict):
        return NAN
    return values.get(name, NAN)


def analyze(rec: Recording) -> Analysis:
    s = rec.summary
    params = s.get('parametros') or {}
    camera = s.get('camera') or {}
    goals = center_goals(s)
    outcome = s.get('desfecho', '')
    row = {
        'rotulo': s.get('rotulo', os.path.basename(os.path.dirname(rec.folder))),
        'repeticao': s.get('repeticao', NAN),
        'pasta': os.path.basename(rec.folder.rstrip('/')),
        'origem': s.get('origem', ''),
        'equipamento': s.get('equipamento', ''),
        'modo': s.get('modo', ''),
        'desfecho': outcome,
        'sucesso': outcome in SUCCESS_OUTCOMES,
        'inicio': s.get('inicio', ''),
        'duracao_s': s.get('duracao_s', NAN),
        'modo_imagem': s.get('modo_imagem', ''),
        'controller': _param(params, 'visual_servo_node', 'controller'),
        'max_vel_deg_s': _param(params, 'visual_servo_node', 'max_vel_deg_s'),
        'f_x': camera.get('fx', NAN),
        'f_y': camera.get('fy', NAN),
        'goals_center': len(goals),
        'readquisicoes': NAN if s.get('readquisicoes') is None else s['readquisicoes'],
    }
    if isinstance(row['controller'], float):
        row['controller'] = ''
    for axis in ('pan', 'tilt'):
        for gain in ('kp', 'ki', 'kd'):
            row[f'{axis}_{gain}'] = _param(params, 'visual_servo_node', f'{axis}.{gain}')
    row.update(dict.fromkeys(METRIC_COLUMNS, NAN))

    a = Analysis(rec=rec, row=row, goal_starts=[g['t_inicio'] for g in goals])
    if row['origem'] == SOURCE_INSPECTION:
        a.tolerance_px = _param(params, 'inspection_manager', 'tolerance_px')

    row['t_b_s'], row['t_i_s'] = search_times(rec.events, rec.servo)
    row['saturacao_pct'] = saturation_pct(rec.servo)
    row.update(detector_stats(rec.timing))

    goal = principal_goal(goals, rec.feedback)
    a.goal = goal
    a.segment = servo_segment(rec.servo, goal)
    vis = visible(a.segment)
    if goal is None or vis is None or rows(vis) == 0:
        return a

    a.t0 = float(vis['t'][0])
    end = float(vis['t'][-1])
    did_converge = converged(rec.feedback, goal)
    if did_converge and goal.get('convergence_time_s') is not None:
        row['t_c_s'] = float(goal['convergence_time_s'])
        row['e_r_px'] = NAN if goal.get('final_error_px') is None else float(goal['final_error_px'])
        a.hold_start = a.t0 + row['t_c_s']
    centered = np.nonzero(vis['centered'] == 1)[0]
    if len(centered):
        a.t_centered = float(vis['t'][centered[0]])

    track = row['modo'] == MODE_TRACK
    iae_end = a.hold_start if (not track and not math.isnan(a.hold_start)) else end
    row['iae_px_s'] = iae(vis['t'], vis['error_px'], a.t0, iae_end)

    step_end = a.t_centered if not math.isnan(a.t_centered) else end
    step = vis['t'] <= step_end
    row['mp_pan_deg'], row['mp_pan_pct'] = overshoot(vis['theta_pan'][step])
    row['mp_tilt_deg'], row['mp_tilt_pct'] = overshoot(vis['theta_tilt'][step])

    if track:
        row['e_d_px'] = tracking_error(rec.feedback, goal)
    return a


def analyze_recording(folder: str) -> Analysis:
    return analyze(load_recording(folder))


def recording_metrics(folder: str) -> dict:
    """Linha do metricas.csv de uma pasta de gravação."""
    return analyze_recording(folder).row


# ---------------- Resumo por rótulo ----------------
def _number(value) -> float:
    if isinstance(value, bool):
        return float(value)
    try:
        return float(value)
    except (TypeError, ValueError):
        return NAN


def summarize(metric_rows: list) -> list:
    """
    Linhas do resumo_por_rotulo.csv (formato longo). As gravações
    interrompidas ficam fora das estatísticas e só são contadas.
    """
    out = []
    labels = sorted({r['rotulo'] for r in metric_rows})
    for label in labels:
        group = [r for r in metric_rows if r['rotulo'] == label]
        valid = [r for r in group if r['desfecho'] != OUTCOME_INTERRUPTED]
        out.append({'rotulo': label, 'metrica': 'gravacoes', 'n': len(group)})
        out.append({'rotulo': label, 'metrica': 'interrompidas', 'n': len(group) - len(valid)})
        for metric in SUMMARY_METRICS:
            values = np.array([_number(r[metric]) for r in valid], dtype=float)
            values = values[~np.isnan(values)]
            line = {'rotulo': label, 'metrica': metric, 'n': len(values)}
            if len(values):
                line.update(media=float(np.mean(values)), minimo=float(np.min(values)),
                            maximo=float(np.max(values)),
                            desvio=float(np.std(values, ddof=1)) if len(values) > 1 else NAN)
            out.append(line)
    return out


# ---------------- Escrita ----------------
def _cell(value):
    if isinstance(value, bool):
        return int(value)
    if isinstance(value, float):
        return '' if math.isnan(value) else format(value, '.6g')
    return value


def write_csv(path: str, columns, lines: list):
    """CSV com cabeçalho; NaN e chaves ausentes viram célula vazia."""
    with open(path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(columns)
        for line in lines:
            writer.writerow([_cell(line.get(c, NAN)) for c in columns])
