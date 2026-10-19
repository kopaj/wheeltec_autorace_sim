"""Offline inspection of one trial's reference, trajectory and error."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial_directory')
    args = parser.parse_args()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    folder = Path(args.trial_directory).expanduser()
    reference = json.loads((folder/'reference.json').read_text())
    with (folder/'trajectory.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        parser.error('No trajectory samples')
    p = np.array(reference['centerline_world_xy_m'])
    p = np.vstack([p, p[0]])
    t = np.array([float(r['t_sim_s']) for r in rows])
    t -= t[0]
    x, y, error = (np.array([float(r[k]) for r in rows]) for k in ('x_m', 'y_m', 'cte_m'))
    # Do not draw a connecting trajectory across missing/jump intervals.
    bad = np.array([r['quality'] not in ('FIRST', 'OK') for r in rows])
    x[bad] = np.nan
    y[bad] = np.nan
    error[bad] = np.nan
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
    axes[0].plot(p[:, 0], p[:, 1], '--', color='0.5', label='Reference')
    axes[0].plot(x, y, color='#0072B2', label='Ground truth')
    axes[0].scatter(*reference['gate_world_xy_m'], marker='x', color='black', label='Lap gate')
    axes[0].set(xlabel='World x [m]', ylabel='World y [m]', title='Rear axle midpoint')
    axes[0].axis('equal')
    axes[0].legend()
    axes[1].plot(t, error, color='#0072B2')
    for sign in (-1, 1):
        axes[1].axhline(sign*reference['lane_width_m']/2, color='#D55E00', ls='--')
    axes[1].set(xlabel='Simulation time from start [s]', ylabel='Signed lateral error [m]',
                title='Dashed lines: nominal lane boundaries')
    for ax in axes:
        ax.grid(alpha=0.25)
    path = folder/'tracking.png'
    fig.savefig(path, dpi=180)
    plt.close(fig)
    print(path)
