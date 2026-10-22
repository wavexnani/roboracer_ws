#!/usr/bin/env python3
"""
Generate publication-quality plots for the Sweet Spot Parameter Certification (Tier 1, 2, 3).
"""

import os
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

EXP_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(EXP_DIR, 'data', 'exp1_3_sweet_spot')
PLOTS_DIR = os.path.join(EXP_DIR, 'plots')
RESULTS_JSON = os.path.join(DATA_DIR, 'certification_results.json')

os.makedirs(PLOTS_DIR, exist_ok=True)
plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
matplotlib.rcParams['font.family'] = 'sans-serif'
matplotlib.rcParams['font.size'] = 11

COLORS = {
    'MPC_Controller': '#7B1FA2',
    'Stanley_Controller': '#388E3C',
    'Pure_Pursuit': '#1976D2',
    'Follow_The_Gap': '#FFA000'
}

LABELS = {
    'MPC_Controller': 'LTV-MPC (Certified)',
    'Stanley_Controller': 'Stanley (Certified)',
    'Pure_Pursuit': 'Pure Pursuit (Speed-Adaptive)',
    'Follow_The_Gap': 'Follow-the-Gap (Certified)'
}


def load_data():
    with open(RESULTS_JSON, 'r') as f:
        return json.load(f)


def generate_plots(data):
    # -------------------------------------------------------------
    # Fig 11: Tier 1 Multi-Track Generalization (10 Laps)
    # -------------------------------------------------------------
    t1_data = [d for d in data if d.get('tier') == 'Tier1_Generalization']
    tracks = ['Levine', 'Spielberg', 'Monza']
    ctrls = ['MPC_Controller', 'Stanley_Controller', 'Pure_Pursuit', 'Follow_The_Gap']

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5.5), dpi=300)
    x = np.arange(len(tracks))
    width = 0.20

    # Cross-Track Error RMSE
    for i, ctrl in enumerate(ctrls):
        rmse_vals = []
        for trk in tracks:
            match = [d for d in t1_data if d['controller'] == ctrl and d['track'] == trk]
            rmse_vals.append(match[0]['cross_track_rmse_m'] if match else np.nan)
        ax1.bar(x + (i - 1.5) * width, rmse_vals, width, label=LABELS[ctrl], color=COLORS[ctrl], edgecolor='black', linewidth=0.7)

    ax1.set_title('(a) Cross-Track Error RMSE Across Tracks (10 Laps)', fontweight='bold')
    ax1.set_ylabel('Cross-Track RMSE (m) [Lower is Better]')
    ax1.set_xticks(x)
    ax1.set_xticklabels(tracks, fontweight='bold')
    ax1.legend(loc='upper right', frameon=True, facecolor='white', framealpha=0.9)

    # Lap Times
    for i, ctrl in enumerate(ctrls):
        lap_vals = []
        for trk in tracks:
            match = [d for d in t1_data if d['controller'] == ctrl and d['track'] == trk]
            lap_vals.append(match[0]['mean_lap_s'] if match else np.nan)
        ax2.bar(x + (i - 1.5) * width, lap_vals, width, label=LABELS[ctrl], color=COLORS[ctrl], edgecolor='black', linewidth=0.7)

    ax2.set_title('(b) Mean Lap Times Across Tracks (10 Laps)', fontweight='bold')
    ax2.set_ylabel('Mean Lap Time (s)')
    ax2.set_xticks(x)
    ax2.set_xticklabels(tracks, fontweight='bold')
    ax2.legend(loc='upper left', frameon=True, facecolor='white', framealpha=0.9)

    fig.suptitle('Tier 1: Multi-Track Generalization & Endurance Certification (10 Laps Each)', fontsize=14, fontweight='bold', y=1.02)
    fig.tight_layout()
    p11 = os.path.join(PLOTS_DIR, 'fig11_sweet_spot_multitrack.png')
    fig.savefig(p11)
    plt.close(fig)
    print(f"Saved: {p11}")

    # -------------------------------------------------------------
    # Fig 12: Tier 2 Fine Grid Validation (MPC & Stanley on Spielberg)
    # -------------------------------------------------------------
    t2_data = [d for d in data if d.get('tier') == 'Tier2_FineGrid']
    mpc_grid = [d for d in t2_data if d['controller'] == 'MPC_Controller']
    stanley_grid = [d for d in t2_data if d['controller'] == 'Stanley_Controller']

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5.5), dpi=300)

    # MPC Grid: RMSE vs Steer Slew
    if mpc_grid:
        for d in mpc_grid:
            ax1.scatter(d['steer_rate_rms_radps'], d['cross_track_rmse_m'], s=140, c=COLORS['MPC_Controller'], edgecolor='black', zorder=4)
            ax1.annotate(d['config'], (d['steer_rate_rms_radps'], d['cross_track_rmse_m']), textcoords="offset points", xytext=(8, -3), fontsize=9, fontweight='bold')
        ax1.set_title('(a) MPC Fine Grid Pareto Frontier', fontweight='bold')
        ax1.set_xlabel('Steering Slew Rate RMS (rad/s) [Smoothness]')
        ax1.set_ylabel('Cross-Track RMSE (m) [Precision]')

    # Stanley Grid: RMSE vs Reversals
    if stanley_grid:
        for d in stanley_grid:
            ax2.scatter(d['steer_reversals'], d['cross_track_rmse_m'], s=140, c=COLORS['Stanley_Controller'], edgecolor='black', zorder=4)
            ax2.annotate(d['config'], (d['steer_reversals'], d['cross_track_rmse_m']), textcoords="offset points", xytext=(8, -3), fontsize=9, fontweight='bold')
        ax2.set_title('(b) Stanley Fine Grid Pareto Frontier', fontweight='bold')
        ax2.set_xlabel('Steering Sign Reversals (5 Laps) [Chatter]')
        ax2.set_ylabel('Cross-Track RMSE (m) [Precision]')

    fig.suptitle('Tier 2: Fine Grid Neighborhood Validation (Pareto Optimality on Spielberg)', fontsize=14, fontweight='bold', y=1.02)
    fig.tight_layout()
    p12 = os.path.join(PLOTS_DIR, 'fig12_fine_grid_optimization.png')
    fig.savefig(p12)
    plt.close(fig)
    print(f"Saved: {p12}")

    # -------------------------------------------------------------
    # Fig 13: Tier 3 Tyre Friction Robustness (mu = 0.80, 1.04, 1.20)
    # -------------------------------------------------------------
    t3_data = [d for d in data if d.get('tier') == 'Tier3_FrictionStress']
    if t3_data:
        mus = [0.80, 1.04, 1.20]
        mpc_mu = [d for d in t3_data if d['controller'] == 'MPC_Controller']
        stanley_mu = [d for d in t3_data if d['controller'] == 'Stanley_Controller']

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5), dpi=300)

        # Tracking Error vs Friction
        ax1.plot([d['mu'] for d in mpc_mu], [d['cross_track_rmse_m'] for d in mpc_mu], 'o-', color=COLORS['MPC_Controller'], linewidth=2.5, markersize=8, label='MPC Controller')
        ax1.plot([d['mu'] for d in stanley_mu], [d['cross_track_rmse_m'] for d in stanley_mu], 's--', color=COLORS['Stanley_Controller'], linewidth=2.5, markersize=8, label='Stanley Controller')
        ax1.set_title('(a) Cross-Track RMSE vs Tyre Friction $\mu$', fontweight='bold')
        ax1.set_xlabel('Tyre Friction Coefficient $\mu$')
        ax1.set_ylabel('Cross-Track RMSE (m)')
        ax1.set_xticks(mus)
        ax1.legend(frameon=True, facecolor='white', framealpha=0.9)

        # Lap Time vs Friction
        ax2.plot([d['mu'] for d in mpc_mu], [d['mean_lap_s'] for d in mpc_mu], 'o-', color=COLORS['MPC_Controller'], linewidth=2.5, markersize=8, label='MPC Controller')
        ax2.plot([d['mu'] for d in stanley_mu], [d['mean_lap_s'] for d in stanley_mu], 's--', color=COLORS['Stanley_Controller'], linewidth=2.5, markersize=8, label='Stanley Controller')
        ax2.set_title('(b) Lap Time vs Tyre Friction $\mu$', fontweight='bold')
        ax2.set_xlabel('Tyre Friction Coefficient $\mu$')
        ax2.set_ylabel('Mean Lap Time (s)')
        ax2.set_xticks(mus)
        ax2.legend(frameon=True, facecolor='white', framealpha=0.9)

        fig.suptitle('Tier 3: Tyre Friction Stress Testing ($\mu = 0.80, 1.04, 1.20$ on Spielberg)', fontsize=14, fontweight='bold', y=1.02)
        fig.tight_layout()
        p13 = os.path.join(PLOTS_DIR, 'fig13_friction_robustness.png')
        fig.savefig(p13)
        plt.close(fig)
        print(f"Saved: {p13}")


if __name__ == '__main__':
    data = load_data()
    generate_plots(data)
