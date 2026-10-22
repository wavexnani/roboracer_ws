#!/usr/bin/env python3
"""
Generate publication-quality comparative plots and markdown report
for Phase 1: Controller Benchmarking (Experiment 1.1).
"""

import os
import json
import csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

EXP_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(EXP_DIR, 'data', 'exp1_1_baseline')
PLOTS_DIR = os.path.join(EXP_DIR, 'plots')
SUMMARY_CSV = os.path.join(DATA_DIR, 'baseline_summary.csv')
RESULTS_JSON = os.path.join(DATA_DIR, 'baseline_results.json')
REPORT_MD = os.path.join(EXP_DIR, 'EXPERIMENT_26_PHASE1_REPORT.md')

os.makedirs(PLOTS_DIR, exist_ok=True)

# Aesthetic styling
plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
matplotlib.rcParams['font.family'] = 'sans-serif'
matplotlib.rcParams['font.size'] = 11
matplotlib.rcParams['axes.titlesize'] = 14
matplotlib.rcParams['axes.labelsize'] = 12
matplotlib.rcParams['figure.titlesize'] = 16

COLORS = {
    'Wall_Following': '#E64A19',      # Deep Orange
    'Follow_The_Gap': '#FFA000',      # Amber
    'Pure_Pursuit': '#1976D2',        # Blue
    'Stanley_Controller': '#388E3C',  # Green
    'MPC_Controller': '#7B1FA2'       # Purple
}

LABELS = {
    'Wall_Following': 'Wall Follow (PID)',
    'Follow_The_Gap': 'Follow-the-Gap (FTG)',
    'Pure_Pursuit': 'Pure Pursuit (PP)',
    'Stanley_Controller': 'Stanley Controller',
    'MPC_Controller': 'LTV-MPC'
}

CONTROLLERS = ['Wall_Following', 'Follow_The_Gap', 'Pure_Pursuit', 'Stanley_Controller', 'MPC_Controller']


def load_data():
    with open(RESULTS_JSON, 'r') as f:
        trials = json.load(f)
    return trials


def generate_plots(trials):
    tracks = sorted(list(set(t['track'] for t in trials)))
    
    # -------------------------------------------------------------
    # Plot 1: Lap Completion Rate by Controller & Track (10-lap trials)
    # -------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(10, 5.5), dpi=300)
    x = np.arange(len(tracks))
    width = 0.16
    
    for i, ctrl in enumerate(CONTROLLERS):
        rates = []
        for trk in tracks:
            # find 10-lap trial
            match = [t for t in trials if t['track'] == trk and t['controller'] == ctrl and t['target_laps'] == 10]
            if match:
                rates.append(match[0]['completion_rate_pct'])
            else:
                rates.append(0.0)
        ax.bar(x + (i - 2) * width, rates, width, label=LABELS[ctrl], color=COLORS[ctrl], edgecolor='black', linewidth=0.7)

    ax.set_title('Lap Completion Rate Across Tracks (10-Lap Stress Test)', pad=15, fontweight='bold')
    ax.set_ylabel('Completion Rate (%)')
    ax.set_xticks(x)
    ax.set_xticklabels(tracks, fontweight='bold')
    ax.set_ylim(0, 115)
    ax.axhline(100, color='gray', linestyle='--', linewidth=0.8, alpha=0.7)
    ax.legend(frameon=True, facecolor='white', framealpha=0.9, loc='upper left')
    fig.tight_layout()
    p1 = os.path.join(PLOTS_DIR, 'fig1_completion_rate.png')
    fig.savefig(p1)
    plt.close(fig)
    print(f"Saved: {p1}")

    # -------------------------------------------------------------
    # Plot 2: Mean Lap Time Comparison (Successful Runs, 10-lap)
    # -------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(10, 5.5), dpi=300)
    for i, ctrl in enumerate(CONTROLLERS):
        times = []
        for trk in tracks:
            match = [t for t in trials if t['track'] == trk and t['controller'] == ctrl and t['target_laps'] == 10]
            if match and match[0]['completed_laps'] > 0 and match[0]['mean_lap_s'] is not None:
                times.append(match[0]['mean_lap_s'])
            else:
                times.append(np.nan)
        ax.bar(x + (i - 2) * width, times, width, label=LABELS[ctrl], color=COLORS[ctrl], edgecolor='black', linewidth=0.7)

    ax.set_title('Mean Lap Time Across Tracks (10-Lap Evaluation)', pad=15, fontweight='bold')
    ax.set_ylabel('Mean Lap Time (seconds)')
    ax.set_xticks(x)
    ax.set_xticklabels(tracks, fontweight='bold')
    ax.legend(frameon=True, facecolor='white', framealpha=0.9, loc='upper left')
    fig.tight_layout()
    p2 = os.path.join(PLOTS_DIR, 'fig2_lap_times.png')
    fig.savefig(p2)
    plt.close(fig)
    print(f"Saved: {p2}")

    # -------------------------------------------------------------
    # Plot 3: Cross-Track Error RMSE (Tracking Precision)
    # -------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(10, 5.5), dpi=300)
    for i, ctrl in enumerate(CONTROLLERS):
        ct_errors = []
        for trk in tracks:
            match = [t for t in trials if t['track'] == trk and t['controller'] == ctrl and t['target_laps'] == 10]
            if match and match[0]['completed_laps'] > 0:
                ct_errors.append(match[0]['cross_track_rmse_m'])
            else:
                ct_errors.append(np.nan)
        ax.bar(x + (i - 2) * width, ct_errors, width, label=LABELS[ctrl], color=COLORS[ctrl], edgecolor='black', linewidth=0.7)

    ax.set_title('Cross-Track Error RMSE (Trajectory Tracking Precision)', pad=15, fontweight='bold')
    ax.set_ylabel('Cross-Track RMSE (meters) [Lower is Better]')
    ax.set_xticks(x)
    ax.set_xticklabels(tracks, fontweight='bold')
    ax.legend(frameon=True, facecolor='white', framealpha=0.9, loc='upper left')
    fig.tight_layout()
    p3 = os.path.join(PLOTS_DIR, 'fig3_cross_track_error.png')
    fig.savefig(p3)
    plt.close(fig)
    print(f"Saved: {p3}")

    # -------------------------------------------------------------
    # Plot 4: Steering Smoothness (RMS Steering Slew Rate)
    # -------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(10, 5.5), dpi=300)
    for i, ctrl in enumerate(CONTROLLERS):
        slew_rates = []
        for trk in tracks:
            match = [t for t in trials if t['track'] == trk and t['controller'] == ctrl and t['target_laps'] == 10]
            if match and match[0]['completed_laps'] > 0:
                slew_rates.append(match[0].get('steering_rate_rms_radps', match[0].get('steer_rate_rms_radps', 0.0)))
            else:
                slew_rates.append(np.nan)
        ax.bar(x + (i - 2) * width, slew_rates, width, label=LABELS[ctrl], color=COLORS[ctrl], edgecolor='black', linewidth=0.7)

    ax.set_title('Steering Slew Rate RMS (Actuator Chattering / Control Smoothness)', pad=15, fontweight='bold')
    ax.set_ylabel('Steer Rate RMS (rad/s) [Lower is Smoother]')
    ax.set_xticks(x)
    ax.set_xticklabels(tracks, fontweight='bold')
    ax.legend(frameon=True, facecolor='white', framealpha=0.9, loc='upper left')
    fig.tight_layout()
    p4 = os.path.join(PLOTS_DIR, 'fig4_steering_smoothness.png')
    fig.savefig(p4)
    plt.close(fig)
    print(f"Saved: {p4}")

    # -------------------------------------------------------------
    # Plot 5: Comprehensive Trade-off Radar Chart
    # -------------------------------------------------------------
    categories = ['Completion\nRate', 'Speed\nProfile', 'Tracking\nAccuracy', 'Steering\nSmoothness', 'Safety\nClearance']
    N_cat = len(categories)
    angles = [n / float(N_cat) * 2 * np.pi for n in range(N_cat)]
    angles += angles[:1]

    fig, ax = plt.subplots(figsize=(8, 8), subplot_kw=dict(polar=True), dpi=300)
    plt.xticks(angles[:-1], categories, color='black', size=11, fontweight='bold')
    ax.set_rlabel_position(25)
    plt.yticks([0.2, 0.4, 0.6, 0.8, 1.0], ["0.2", "0.4", "0.6", "0.8", "1.0"], color="grey", size=9)
    plt.ylim(0, 1.05)

    # Compute normalized composite scores across all tracks
    for ctrl in CONTROLLERS:
        ctrl_trials = [t for t in trials if t['controller'] == ctrl and t['target_laps'] == 10]
        if not ctrl_trials:
            continue
        
        # 1. Completion rate (0 to 1)
        cr = np.mean([t['completion_rate_pct'] for t in ctrl_trials]) / 100.0
        
        # 2. Speed score (normalized relative to max 4.64 m/s)
        speeds = [t['average_speed_mps'] for t in ctrl_trials if t['completed_laps'] > 0]
        speed_score = np.mean(speeds) / 4.70 if speeds else 0.35
        
        # 3. Accuracy score: 1.0 - (min(rmse, 0.6) / 0.6)
        rmses = [t['cross_track_rmse_m'] for t in ctrl_trials if t['completed_laps'] > 0]
        acc_score = 1.0 - (np.mean(rmses) / 0.65) if rmses else 0.0
        acc_score = max(0.05, min(1.0, acc_score))
        
        # 4. Smoothness score: 1.0 - min(steer_rms, 5.0) / 5.0
        slew_vals = [t.get('steering_rate_rms_radps', t.get('steer_rate_rms_radps', 0.0)) for t in ctrl_trials if t['completed_laps'] > 0]
        smooth_score = 1.0 - (np.mean(slew_vals) / 5.5) if slew_vals else 0.0
        smooth_score = max(0.05, min(1.0, smooth_score))
        
        # 5. Safety clearance: min_lidar / 0.40
        lidars = [t['min_lidar_distance_m'] for t in ctrl_trials if t['completed_laps'] > 0]
        safety_score = np.mean(lidars) / 0.40 if lidars else 0.10
        safety_score = max(0.05, min(1.0, safety_score))

        values = [cr, speed_score, acc_score, smooth_score, safety_score]
        values += values[:1]

        ax.plot(angles, values, linewidth=2, linestyle='solid', label=LABELS[ctrl], color=COLORS[ctrl])
        ax.fill(angles, values, color=COLORS[ctrl], alpha=0.15)

    ax.set_title('Overall Controller Trade-Off Radar (Normalized Performance)', size=14, fontweight='bold', pad=25)
    ax.legend(loc='upper right', bbox_to_anchor=(1.25, 1.15), frameon=True, facecolor='white', framealpha=0.9)
    fig.tight_layout()
    p5 = os.path.join(PLOTS_DIR, 'fig5_controller_radar.png')
    fig.savefig(p5)
    plt.close(fig)
    print(f"Saved: {p5}")


if __name__ == '__main__':
    data = load_data()
    generate_plots(data)

