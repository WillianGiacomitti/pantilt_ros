"""Gravações sintéticas do experiment_recorder para os testes da análise."""

import json
import math
import os

import pytest

from pantilt_experiment.csv_writer import CsvWriter
from pantilt_experiment.session import csv_columns

F_PX = 800.0


def servo_row(t, ex, ey=0.0, visible=True, centered=False, sat=False, cmd=0.1, t_rx=None,
              mode=0):
    """Linha do servo.csv (θ = atan(e/f), como no visual_servo_node)."""
    error_px = math.hypot(ex, ey) if visible else 0.0
    return [t, t + 0.05 if t_rx is None else t_rx, 'pid', mode, visible, ex, ey, error_px,
            math.atan(ex / F_PX), math.atan(ey / F_PX), 0.1, False,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, cmd, cmd, sat, False, cmd, cmd, centered, 0.0, 0.0]


def write_recording(folder, summary, **tables):
    """Pasta de gravação com resumo.json e os CSVs dados (nome -> lista de linhas)."""
    os.makedirs(folder)
    writer = CsvWriter(folder, csv_columns(), 10000)
    for name, lines in tables.items():
        for line in lines:
            writer.write(name, line)
    writer.close()
    with open(os.path.join(folder, 'resumo.json'), 'w', encoding='utf-8') as f:
        json.dump(summary, f)
    return folder


def summary(label='kp0.8', rep=1, mode='center', outcome='centralizado', source='inspecao',
            goals=None, readq=0):
    return {
        'rotulo': label, 'repeticao': rep, 'origem': source, 'equipamento': 'garrafa',
        'modo': mode, 'desfecho': outcome, 'inicio': '2026-10-22T10:00:00.000',
        'duracao_s': 5.0, 'modo_imagem': 'none', 'readquisicoes': readq,
        'center': goals or [], 'scan': [],
        'camera': {'fx': F_PX, 'fy': F_PX, 'cx': 320.0, 'cy': 240.0},
        'parametros': {
            'visual_servo_node': {'controller': 'pid', 'pan.kp': 0.8, 'pan.ki': 0.0,
                                  'pan.kd': 0.0, 'tilt.kp': 0.8, 'tilt.ki': 0.0,
                                  'tilt.kd': 0.0, 'max_vel_deg_s': 20.0},
            'inspection_manager': {'tolerance_px': 20.0, 'hold_time_s': 1.0},
        },
    }


def goal(key, t_start, t_end, t_c=0.5, e_r=4.0, success=True):
    return {'goal': key, 't_inicio': t_start, 't_fim': t_end, 'status': 'sucesso',
            'message': '', 'success': success, 'convergence_time_s': t_c,
            'final_error_px': e_r}


def center_recording(folder, label='kp0.8', rep=1, overshoot_px=-20.0):
    """
    Inspeção CENTER: um goal de 1,0 a 4,0 s. Erro x: 100 px em t=1,1, rampa até
    0 em t=1,6, sobressinal overshoot_px em 1,7, dentro da tolerância de 1,8 em
    diante; centered=true em 2,8 (permanência de 1 s a partir de 1,8).
    """
    errors = {1.1: 100.0, 1.2: 80.0, 1.3: 60.0, 1.4: 40.0, 1.5: 20.0, 1.6: 0.0,
              1.7: overshoot_px}
    servo = []
    t = 1.1
    while t < 3.95:
        t = round(t, 2)
        ex = errors.get(t, 5.0)
        servo.append(servo_row(t, ex, centered=t >= 2.8, sat=t <= 1.2))
        t += 0.1
    feedback = [[r[1], r[1], 'aaaa0001', r[5], r[6], r[7], r[4], r[24]] for r in servo]
    events = [[0.0, 0.0, 'gravador', 'inicio', ''], [0.05, 0.06, 'inspecao', 'SEARCHING', ''],
              [0.9, 0.91, 'inspecao', 'CENTERING', ''], [4.1, 4.11, 'inspecao', 'IDLE', '']]
    timing = [[1.0 + 0.1 * i, 1.05 + 0.1 * i, 70.0 + i, 110.0 + i, 1] for i in range(20)]
    joints = [[0.05 * i, 0.05 * i, 0.001 * i, 0.0, 0.0, 0.0] for i in range(90)]
    s = summary(label=label, rep=rep,
                goals=[goal('aaaa0001', 1.0, 4.0, t_c=0.7, e_r=5.0)])
    return write_recording(folder, s, servo=servo, center_feedback=feedback, events=events,
                           timing=timing, joints=joints)


def track_recording(folder, label='kp0.8', rep=2):
    """
    Inspeção TRACK com readquisição: goal 1 (0,5 a 1,5 s) perde o alvo sem
    convergir; goal 2 (2,0 a 5,0 s) converge (centered em 3,5) e segue rastreando.
    """
    servo, feedback = [], []
    for i in range(9):
        t = round(0.6 + 0.1 * i, 2)
        servo.append(servo_row(t, 60.0, mode=1))
        feedback.append([t, t, 'bbbb0001', 60.0, 0.0, 60.0, True, False])
    for i in range(29):
        t = round(2.1 + 0.1 * i, 2)
        ex = max(5.0, 50.0 - 10.0 * i)
        centered = t >= 3.5
        vis = not (3.9 <= t <= 4.0)          # alvo some por 2 amostras
        servo.append(servo_row(t, ex if vis else 0.0, visible=vis, centered=centered, mode=1))
        feedback.append([t, t, 'bbbb0002', ex if vis else 0.0, 0.0, ex if vis else 0.0, vis,
                         centered])
    events = [[0.1, 0.1, 'inspecao', 'SEARCHING', ''], [0.4, 0.4, 'inspecao', 'CENTERING', ''],
              [1.6, 1.6, 'inspecao', 'SEARCHING', ''], [1.9, 1.9, 'inspecao', 'CENTERING', ''],
              [3.6, 3.6, 'inspecao', 'TRACKING', '']]
    goals = [goal('bbbb0001', 0.5, 1.5, t_c=0.0, e_r=0.0, success=False),
             goal('bbbb0002', 2.0, 5.0, t_c=0.4, e_r=5.0, success=False)]
    s = summary(label=label, rep=rep, mode='track', outcome='rastreou', goals=goals, readq=1)
    return write_recording(folder, s, servo=servo, center_feedback=feedback, events=events)


@pytest.fixture
def ensaios(tmp_path):
    """Pasta raiz com duas gravações do rótulo kp0.8 (CENTER e TRACK)."""
    root = tmp_path / 'ensaios'
    center_recording(str(root / 'kp0.8' / '001_20261022_100000_garrafa_center'))
    track_recording(str(root / 'kp0.8' / '002_20261022_100100_garrafa_track'))
    return root
