#!/usr/bin/env python3
"""
Per-biome RMSE table for the altitude fusion stages.

Reads mission data pickles produced by mission_node.py (one per scenario)
and computes altitude RMSE for each fusion stage against ground truth,
grouped by biome. Outputs a markdown table by default; --latex produces
a paper-ready LaTeX table.

Usage:
  python3 scripts/evaluation/compute_rmse_table.py
  python3 scripts/evaluation/compute_rmse_table.py --logs-dir logs
  python3 scripts/evaluation/compute_rmse_table.py --stage-only stage2b
  python3 scripts/evaluation/compute_rmse_table.py --latex > table.tex

Method:
  1. Pick reference: Gazebo ground truth (gt_*) if present, else fall back
     to MAVROS pose (traj_*) with a warning flag in the output.
  2. Linear-interpolate the reference to the diag_t timebase so each
     fusion sample has a paired reference altitude.
  3. Classify each sample by biome from its (x, y) position against
     BIOME_META centres+radii. Below 10m altitude = GROUND, excluded.
  4. RMSE = sqrt(mean((fused - ref)^2)) per (scenario, biome, stage).

Caveat worth noting in the paper:
  Stage-2A ACF is a LiDAR-dominant AGL estimator. Over hilly biomes
  (TERRAIN) it reports height-above-ground, which is legitimately
  different from the ASL ground truth. So a large ACF RMSE over TERRAIN
  is frame-of-reference mismatch, not filter error. The VEGETATION
  row (flat ground) is where the comparison is apples-to-apples.
"""

import argparse
import glob
import math
import os
import pickle
import sys
from pathlib import Path

import numpy as np


# Match mission_node.BIOME_META. Keep in sync.
BIOME_META = {
    'CITY':       {'cx':  0.0, 'cy': -100.0, 'alt': 70.0, 'radius': 35.0},
    'VEGETATION': {'cx': 90.0, 'cy':    0.0, 'alt': 30.0, 'radius': 35.0},
    'TERRAIN':    {'cx':  0.0, 'cy':  150.0, 'alt': 75.0, 'radius': 35.0},
}

# Samples below this altitude are takeoff/landing — fusion isn't really
# tracking yet, so they're excluded from RMSE.
GROUND_ALT_THRESHOLD = 10.0

STAGE_CHANNELS = {
    'stage1':  ('diag_baro', 'Stage-1 EKF-1 (IMU+Baro)'),
    'stage2a': ('diag_acf',  'Stage-2A ACF'),
    'stage2b': ('diag_ekf2', 'Stage-2B EKF-2 IAKF'),
}


def load_pickle(path):
    try:
        with open(path, 'rb') as fp:
            return pickle.load(fp)
    except Exception as e:
        print(f"WARN: failed to load {path}: {e}", file=sys.stderr)
        return None


def pick_reference(pkl):
    # Prefer ground truth; fall back to MAVROS pose.
    if len(pkl.get('gt_t', [])) > 10:
        return (np.asarray(pkl['gt_t']),
                np.asarray(pkl['gt_x']),
                np.asarray(pkl['gt_y']),
                np.asarray(pkl['gt_z']),
                'gazebo_gt')
    return (np.asarray(pkl['traj_t']),
            np.asarray(pkl['traj_x']),
            np.asarray(pkl['traj_y']),
            np.asarray(pkl['traj_z']),
            'mavros_pose_fallback')


def classify_segment(x, y, z):
    if z < GROUND_ALT_THRESHOLD:
        return 'GROUND'
    for name, meta in BIOME_META.items():
        if math.hypot(x - meta['cx'], y - meta['cy']) <= meta['radius']:
            return name
    return 'TRANSIT'


def segment_mask_for_positions(xs, ys, zs):
    labels = np.empty(len(xs), dtype=object)
    for i, (x, y, z) in enumerate(zip(xs, ys, zs)):
        labels[i] = classify_segment(x, y, z)
    return labels


def compute_scenario_rmse(pkl):
    ref_t, ref_x, ref_y, ref_z, ref_source = pick_reference(pkl)
    diag_t = np.asarray(pkl.get('diag_t', []))
    if len(diag_t) < 10 or len(ref_t) < 10:
        return None, ref_source

    # Interpolate reference onto the fusion diagnostic timebase.
    ref_z_on_diag = np.interp(diag_t, ref_t, ref_z)
    ref_x_on_diag = np.interp(diag_t, ref_t, ref_x)
    ref_y_on_diag = np.interp(diag_t, ref_t, ref_y)

    biome_labels = segment_mask_for_positions(
        ref_x_on_diag, ref_y_on_diag, ref_z_on_diag)

    stage_errors = {}
    for stage_key, (chan, _) in STAGE_CHANNELS.items():
        est = np.asarray(pkl.get(chan, []))
        if len(est) != len(diag_t):
            # Channel length mismatch — skip this stage.
            continue
        stage_errors[stage_key] = est - ref_z_on_diag

    result = {}
    for stage_key, err in stage_errors.items():
        result[stage_key] = {}
        for biome in list(BIOME_META.keys()) + ['TRANSIT', 'ALL']:
            mask = (biome_labels != 'GROUND') if biome == 'ALL' else (biome_labels == biome)
            if mask.sum() < 5:
                result[stage_key][biome] = {'rmse': float('nan'), 'n': int(mask.sum())}
                continue
            e = err[mask]
            result[stage_key][biome] = {
                'rmse': float(np.sqrt(np.mean(e ** 2))),
                'n': int(mask.sum()),
            }

    return result, ref_source


def fmt_cell(rmse, n):
    if n < 5 or math.isnan(rmse):
        return '    —   '
    if n < 50:
        return f'{rmse:5.2f}*'
    return f'{rmse:5.2f}'


def emit_markdown(all_results, scenarios_in_order, stage_key):
    chan, label = STAGE_CHANNELS[stage_key]
    print(f"\n## {label}")
    print("\nRMSE (m), vs ground-truth Z.  Dashes indicate insufficient "
          "samples (<5).  An asterisk (*) indicates fewer than 50 "
          "samples in that biome.\n")
    biomes = ['CITY', 'VEGETATION', 'TERRAIN', 'TRANSIT', 'ALL']
    header = '| Scenario         | Ref     | ' + ' | '.join(f'{b:>10}' for b in biomes) + ' |'
    sep = '|' + '-' * (len(header) - 2) + '|'
    print(header)
    print(sep)
    for scen in scenarios_in_order:
        if scen not in all_results:
            continue
        res, ref_source = all_results[scen]
        if res is None or stage_key not in res:
            row = (f'| {scen:16s} | {ref_source[:6]:6s}  | '
                   + ' | '.join('   —   ' for _ in biomes) + ' |')
        else:
            ref_short = 'gz_gt ' if ref_source == 'gazebo_gt' else '⚠mavr '
            cells = [fmt_cell(res[stage_key].get(b, {'rmse': float('nan'), 'n': 0})['rmse'],
                              res[stage_key].get(b, {'rmse': float('nan'), 'n': 0})['n'])
                     for b in biomes]
            row = (f'| {scen:16s} | {ref_short} | '
                   + ' | '.join(f'{c:>10}' for c in cells) + ' |')
        print(row)


def emit_latex(all_results, scenarios_in_order):
    biomes = ['CITY', 'VEGETATION', 'TERRAIN', 'TRANSIT', 'ALL']
    stages = list(STAGE_CHANNELS.keys())
    print(r"% Auto-generated by compute_rmse_table.py")
    print(r"\begin{table}[h]")
    print(r"\centering")
    print(r"\caption{Altitude RMSE (m) by scenario, biome, and fusion stage. "
          r"Asterisk denotes <50 samples; dash denotes insufficient data.}")
    print(r"\label{tab:rmse}")
    print(r"\begin{tabular}{l|l|" + 'c' * len(biomes) + r"}")
    print(r"\toprule")
    print(r"Scenario & Stage & " + ' & '.join(biomes) + r" \\")
    print(r"\midrule")
    for scen in scenarios_in_order:
        if scen not in all_results:
            continue
        res, _ = all_results[scen]
        if res is None:
            continue
        for i, stage_key in enumerate(stages):
            label = STAGE_CHANNELS[stage_key][1].split('(')[0].strip()
            cells = []
            for b in biomes:
                entry = res[stage_key].get(b, {'rmse': float('nan'), 'n': 0})
                if entry['n'] < 5 or math.isnan(entry['rmse']):
                    cells.append('---')
                elif entry['n'] < 50:
                    cells.append(f'{entry["rmse"]:.2f}*')
                else:
                    cells.append(f'{entry["rmse"]:.2f}')
            scen_cell = scen if i == 0 else ''
            print(f'{scen_cell} & {label} & ' + ' & '.join(cells) + r' \\')
        print(r'\midrule')
    print(r"\bottomrule")
    print(r"\end{tabular}")
    print(r"\end{table}")


def main():
    ap = argparse.ArgumentParser(description="Per-biome altitude RMSE table.")
    ap.add_argument('--logs-dir', default='logs',
                    help='directory containing mission_data_*.pkl (default: logs)')
    ap.add_argument('--latex', action='store_true',
                    help='emit LaTeX table instead of markdown')
    ap.add_argument('--stage-only',
                    help='restrict to one stage: stage1, stage2a, or stage2b')
    args = ap.parse_args()

    pattern = os.path.join(args.logs_dir, 'mission_data_*.pkl')
    files = sorted(glob.glob(pattern))
    if not files:
        print(f"No pickles found at {pattern}", file=sys.stderr)
        print("Is mission_node.py saving them? Check _generate_plots.", file=sys.stderr)
        sys.exit(1)

    print(f"# RMSE report — {len(files)} scenarios found", file=sys.stderr)
    for f in files:
        print(f"  - {f}", file=sys.stderr)

    canonical_order = ['nominal', 'wind', 'drift', 'dropout', 'combined']

    all_results = {}
    for path in files:
        pkl = load_pickle(path)
        if pkl is None:
            continue
        scen = pkl.get('scenario', Path(path).stem)
        # If the same scenario has multiple pickles, keep the latest.
        # Files are sorted ascending so this will naturally win.
        res, ref_source = compute_scenario_rmse(pkl)
        all_results[scen] = (res, ref_source)

    ordered = [s for s in canonical_order if s in all_results]
    extras = sorted(s for s in all_results if s not in canonical_order)
    scenarios_in_order = ordered + extras

    if args.latex:
        emit_latex(all_results, scenarios_in_order)
        return

    print("# Altitude RMSE — per scenario × biome × fusion stage\n")
    print("Reference legend:  `gz_gt ` = Gazebo ground truth (true pose).  "
          "`⚠mavr ` = MAVROS EKF3 fallback (optimistically biased since "
          "EKF3 shares sensor inputs with our fusion stages).\n")
    print(f"Biome definition:  a sample is in biome X if its (x,y) is "
          f"within {BIOME_META['CITY']['radius']:.0f} m of biome X's "
          f"centre.  Samples below {GROUND_ALT_THRESHOLD:.0f} m altitude "
          f"are excluded (takeoff/landing).  TRANSIT = not in any biome.  "
          f"ALL = all flying samples.\n")

    stages = [args.stage_only] if args.stage_only else list(STAGE_CHANNELS.keys())
    for st in stages:
        if st not in STAGE_CHANNELS:
            print(f"Unknown stage '{st}'. Choices: {list(STAGE_CHANNELS.keys())}",
                  file=sys.stderr)
            sys.exit(1)
        emit_markdown(all_results, scenarios_in_order, st)

    print("\n## Notes")
    n_with_gt = sum(1 for s, (r, rs) in all_results.items() if rs == 'gazebo_gt')
    n_total = len(all_results)
    print(f"- {n_with_gt}/{n_total} scenarios have Gazebo ground-truth "
          f"reference; the rest fall back to MAVROS pose (biased, flagged ⚠).")
    if n_with_gt < n_total:
        print("- For ground truth on the remaining runs, ensure "
              "`gz_pose_bridge.py` is running during the mission.")
    print("- Stage-2A ACF RMSE over TERRAIN should be interpreted with "
          "caution: ACF reports AGL, not ASL, so a large RMSE there "
          "reflects the AGL/ASL frame difference over hills, not filter "
          "error. The paper should note this explicitly.")


if __name__ == '__main__':
    main()
