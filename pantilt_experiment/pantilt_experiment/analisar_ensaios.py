#!/usr/bin/env python3
"""
Análise dos ensaios do TCC (docs/architecture.md, seção 4.11 "Análise", e
seção 10): lê as pastas do experiment_recorder e gera em <pasta>/_analise/

  metricas.csv            uma linha por gravação
  resumo_por_rotulo.csv   n, média, desvio, mínimo e máximo por rótulo e métrica
  figuras/                uma figura por gravação e as comparativas (PNG e PDF)

Uso:
  ros2 run pantilt_experiment analisar_ensaios [pasta] [--rotulos r1 r2] [--sem-figuras]

Pastas que começam com "_" e gravações sem resumo.json (em andamento) são
ignoradas. Um erro numa gravação vira aviso e não interrompe as demais.
"""

import argparse
import math
import os
import sys

from pantilt_experiment.analysis import (
    ANALYSIS_DIR, METRICS_CSV_COLUMNS, SUMMARY_COLUMNS, analyze_recording, find_recordings,
    summarize, write_csv,
)

DEFAULT_DIR = '/ros2_ws/ensaios'

# Métricas mostradas no resumo do terminal: (métrica, título, casas decimais)
TERMINAL_METRICS = (
    ('t_c_s', 't_c (s)', 2), ('e_r_px', 'e_r (px)', 1), ('iae_px_s', 'IAE (px·s)', 1),
    ('mp_pan_deg', 'M_p pan (°)', 2), ('mp_tilt_deg', 'M_p tilt (°)', 2),
    ('e_d_px', 'e_d (px)', 1),
)


def _fmt(line: dict, digits: int) -> str:
    mean, std = line.get('media', math.nan), line.get('desvio', math.nan)
    if math.isnan(mean):
        return '—'
    if math.isnan(std):
        return f'{mean:.{digits}f}'
    return f'{mean:.{digits}f} ± {std:.{digits}f}'


def print_summary(summary: list):
    by_label = {}
    for line in summary:
        by_label.setdefault(line['rotulo'], {})[line['metrica']] = line
    for label, metrics in by_label.items():
        total = metrics['gravacoes']['n']
        interrupted = metrics['interrompidas']['n']
        success = metrics['sucesso']
        rate = success.get('media', math.nan)
        rate = '—' if math.isnan(rate) else f'{100 * rate:.0f}%'
        print(f'\n{label}: {total} gravações ({interrupted} interrompidas), sucesso {rate}')
        for key, title, digits in TERMINAL_METRICS:
            line = metrics[key]
            if line['n']:
                print(f'  {title:<14} {_fmt(line, digits)}  (n={line["n"]})')


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description='Métricas e gráficos dos ensaios (pastas do experiment_recorder).')
    parser.add_argument('pasta', nargs='?', default=DEFAULT_DIR,
                        help=f'pasta raiz das gravações (padrão: {DEFAULT_DIR})')
    parser.add_argument('--rotulos', nargs='+', metavar='ROTULO',
                        help='analisa só estes rótulos')
    parser.add_argument('--sem-figuras', action='store_true',
                        help='só os CSVs, sem gerar figuras')
    args = parser.parse_args(argv)

    found, pending = find_recordings(args.pasta, args.rotulos)
    for folder in pending:
        print(f'Aviso: {folder} sem resumo.json (gravação em andamento?): ignorada')
    if not found:
        print(f'Nenhuma gravação encontrada em {args.pasta}')
        return 1

    analyses, failures = [], 0
    for folder in found:
        try:
            analyses.append(analyze_recording(folder))
        except (OSError, ValueError, KeyError, TypeError) as e:
            failures += 1
            print(f'Aviso: {folder}: não foi possível analisar ({type(e).__name__}: {e})')

    out_dir = os.path.join(args.pasta, ANALYSIS_DIR)
    os.makedirs(out_dir, exist_ok=True)
    metric_rows = [a.row for a in analyses]
    summary = summarize(metric_rows)
    write_csv(os.path.join(out_dir, 'metricas.csv'), METRICS_CSV_COLUMNS, metric_rows)
    write_csv(os.path.join(out_dir, 'resumo_por_rotulo.csv'), SUMMARY_COLUMNS, summary)
    print(f'{len(analyses)} gravações analisadas'
          + (f', {failures} com erro' if failures else '') + f' -> {out_dir}')

    if not args.sem_figuras:
        # Importado só aqui: --sem-figuras funciona sem matplotlib
        from pantilt_experiment.plots import plot_comparisons, plot_recording
        fig_dir = os.path.join(out_dir, 'figuras')
        count = 0
        for a in analyses:
            base = os.path.join(fig_dir, a.row['rotulo'], a.row['pasta'])
            try:
                count += bool(plot_recording(a, base))
            except (ValueError, KeyError, TypeError) as e:
                print(f'Aviso: figura de {a.row["pasta"]} não gerada ({type(e).__name__}: {e})')
        plot_comparisons(analyses, fig_dir)
        print(f'{count} figuras por gravação e as comparativas em {fig_dir}')

    print_summary(summary)
    return 0


if __name__ == '__main__':
    sys.exit(main())
