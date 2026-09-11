#!/usr/bin/env python3
"""
Generates publication-grade comparative benchmark plots for:
1. Original Stanley Controller
2. Modified Stanley Controller
3. Baseline MPC Controller
4. Latest MPC + Curvature-Aware Velocity Controller

Produces high-res PNG plots saved to `benchmark_plots/`:
- `velocity_vs_distance_time.png`
- `steering_angle_vs_time.png`
- `crosstrack_error_vs_distance.png`
- `sharp_vs_smooth_turn_performance.png`
- `generalization_unseen_circuits.png`
- `comprehensive_controller_dashboard.png`
"""

import os
import sys
import json
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np

WS_ROOT = '/home/yeswanth/roboracer_ws'
PLOTS_DIR = os.path.join(WS_ROOT, 'benchmark_plots')
os.makedirs(PLOTS_DIR, exist_ok=True)

# Color Palette & Styling
plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.size'] = 11
plt.rcParams['axes.titlesize'] = 13
plt.rcParams['axes.labelsize'] = 11
plt.rcParams['xtick.labelsize'] = 10
plt.rcParams['ytick.labelsize'] = 10
plt.rcParams['legend.fontsize'] = 10
plt.rcParams['figure.titlesize'] = 15

COLORS = {
    'Original_Stanley': '#d95f02',   # Classic Orange
    'Modified_Stanley': '#7570b3',   # Purple
    'Baseline_MPC':     '#e7298a',   # Magenta / Reddish
    'Latest_MPC':       '#1b9e77'    # Emerald Racing Green
}

LABELS = {
    'Original_Stanley': '1. Original Stanley (Classic, a34b5db)',
    'Modified_Stanley': '2. Modified Stanley (Augmented, 37b6afd)',
    'Baseline_MPC':     '3. Baseline MPC (OSQP LTV, b116494)',
    'Latest_MPC':       '4. Latest MPC + Curv-Aware (1e57c15)'
}


def load_data():
    summary_path = os.path.join(WS_ROOT, 'benchmark_results.json')
    telem_path = os.path.join(WS_ROOT, 'benchmark_telemetry.json')
    with open(summary_path, 'r') as f:
        summary = json.load(f)
    with open(telem_path, 'r') as f:
        telem = json.load(f)
    return summary, telem


def plot_velocity_vs_distance_time(telem):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 9), dpi=300)
    track = 'Spielberg'
    
    # 1. Velocity vs Distance
    for ctrl, data in telem[track].items():
        if not data or 'distance' not in data:
            continue
        dist = np.array(data['distance'])
        speed = np.array(data['speed'])
        ax1.plot(dist, speed, label=LABELS.get(ctrl, ctrl), color=COLORS.get(ctrl, '#333'),
                 linewidth=2.2 if ctrl == 'Latest_MPC' else 1.6, alpha=0.9)
        
    ax1.set_title(f'Velocity Profile vs Track Distance — {track} Circuit (Length ~338.9m)', fontweight='bold')
    ax1.set_xlabel('Arc-Length Distance along Circuit [m]')
    ax1.set_ylabel('Longitudinal Velocity [m/s]')
    ax1.set_xlim(0, 340)
    ax1.set_ylim(0, 8.2)
    ax1.axhline(7.5, color='#1b9e77', linestyle='--', alpha=0.5, label='Latest MPC Max Straight Ceiling (7.5 m/s)')
    ax1.legend(loc='lower right', framealpha=0.95)
    ax1.grid(True, linestyle=':', alpha=0.6)

    # 2. Velocity vs Elapsed Time
    for ctrl, data in telem[track].items():
        if not data or 'time' not in data:
            continue
        t = np.array(data['time'])
        speed = np.array(data['speed'])
        ax2.plot(t, speed, label=LABELS.get(ctrl, ctrl), color=COLORS.get(ctrl, '#333'),
                 linewidth=2.2 if ctrl == 'Latest_MPC' else 1.6, alpha=0.9)

    ax2.set_title(f'Velocity vs Elapsed Simulation Time — Lap Duration Comparison ({track})', fontweight='bold')
    ax2.set_xlabel('Time Elapsed [seconds]')
    ax2.set_ylabel('Longitudinal Velocity [m/s]')
    ax2.set_ylim(0, 8.2)
    ax2.legend(loc='upper right', framealpha=0.95)
    ax2.grid(True, linestyle=':', alpha=0.6)

    plt.tight_layout()
    out_file = os.path.join(PLOTS_DIR, 'velocity_vs_distance_time.png')
    plt.savefig(out_file, dpi=300)
    plt.close()
    print(f"[PLOT] Generated: {out_file}")


def plot_steering_angle_vs_time(telem):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 9), dpi=300)
    track = 'Spielberg'
    
    # Time window 0s to 30s covering Turn 1 and approach to Turn 3 hairpin
    for ctrl, data in telem[track].items():
        if not data or 'time' not in data:
            continue
        t = np.array(data['time'])
        steer_deg = np.degrees(np.array(data['steer']))
        steer_rate = np.array(data['steer_rate'])
        
        mask = t <= 35.0
        ax1.plot(t[mask], steer_deg[mask], label=LABELS.get(ctrl, ctrl), color=COLORS.get(ctrl, '#333'),
                 linewidth=2.0 if ctrl == 'Latest_MPC' else 1.5, alpha=0.9)
        ax2.plot(t[mask], steer_rate[mask], label=LABELS.get(ctrl, ctrl), color=COLORS.get(ctrl, '#333'),
                 linewidth=1.8 if ctrl == 'Latest_MPC' else 1.3, alpha=0.85)

    ax1.set_title(f'Steering Angle Response (\u03b4) Over Technical Sector — {track} (Turn 1 to Turn 3 Hairpin)', fontweight='bold')
    ax1.set_xlabel('Time [s]')
    ax1.set_ylabel('Front Steering Angle [\u00b0]')
    ax1.axhline(24.0, color='r', linestyle=':', label='Steering Lock Limit (\u00b124\u00b0)')
    ax1.axhline(-24.0, color='r', linestyle=':')
    ax1.legend(loc='upper right', framealpha=0.95)
    ax1.grid(True, linestyle=':', alpha=0.6)

    ax2.set_title('Steering Slew Rate (|\u0394\u03b4/\u0394t|) — Comparing Jitter/Flutter vs Deadband Polish', fontweight='bold')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Steering Rate [rad/s]')
    ax2.legend(loc='upper right', framealpha=0.95)
    ax2.grid(True, linestyle=':', alpha=0.6)

    plt.tight_layout()
    out_file = os.path.join(PLOTS_DIR, 'steering_angle_vs_time.png')
    plt.savefig(out_file, dpi=300)
    plt.close()
    print(f"[PLOT] Generated: {out_file}")


def plot_crosstrack_error(telem):
    fig, ax = plt.subplots(figsize=(14, 6), dpi=300)
    track = 'Spielberg'
    
    for ctrl, data in telem[track].items():
        if not data or 'distance' not in data:
            continue
        dist = np.array(data['distance'])
        cte = np.abs(np.array(data['cross_track_err']))
        ax.plot(dist, cte, label=LABELS.get(ctrl, ctrl), color=COLORS.get(ctrl, '#333'),
                linewidth=2.0 if ctrl == 'Latest_MPC' else 1.4, alpha=0.85)

    ax.set_title(f'Absolute Cross-Track Tracking Error (|e_ct|) along {track}', fontweight='bold')
    ax.set_xlabel('Circuit Distance [m]')
    ax.set_ylabel('Cross-Track Error [m]')
    ax.set_xlim(0, 340)
    ax.set_ylim(0, 0.40)
    ax.axhline(0.10, color='gray', linestyle='--', alpha=0.6, label='High Precision Boundary (\u00b110 cm)')
    ax.legend(loc='upper right', framealpha=0.95)
    ax.grid(True, linestyle=':', alpha=0.6)

    plt.tight_layout()
    out_file = os.path.join(PLOTS_DIR, 'crosstrack_error_vs_distance.png')
    plt.savefig(out_file, dpi=300)
    plt.close()
    print(f"[PLOT] Generated: {out_file}")


def plot_sharp_vs_smooth_bars(summary):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6), dpi=300)
    
    # Filter for Spielberg results
    res_sp = {r['controller']: r for r in summary if r['track_name'] == 'Spielberg'}
    ctrl_order = ['Original_Stanley', 'Modified_Stanley', 'Baseline_MPC', 'Latest_MPC']
    
    sharp_errs = [res_sp[c]['sharp_curve_avg_cte'] for c in ctrl_order if c in res_sp]
    smooth_errs = [res_sp[c]['smooth_curve_avg_cte'] for c in ctrl_order if c in res_sp]
    
    sharp_speeds = [res_sp[c]['sharp_curve_avg_speed'] for c in ctrl_order if c in res_sp]
    smooth_speeds = [res_sp[c]['smooth_curve_avg_speed'] for c in ctrl_order if c in res_sp]
    
    x = np.arange(len(ctrl_order))
    width = 0.35
    
    # Subplot 1: Tracking Errors
    rects1 = ax1.bar(x - width/2, sharp_errs, width, label='Sharp Curves (|kappa| \u2265 0.15)', color='#e41a1c', alpha=0.85)
    rects2 = ax1.bar(x + width/2, smooth_errs, width, label='Smooth Curves / Straights (|kappa| < 0.05)', color='#377eb8', alpha=0.85)
    ax1.set_ylabel('Mean Cross-Track Error [m]')
    ax1.set_title('Tracking Error: Sharp vs Smooth Curves (Spielberg)', fontweight='bold')
    ax1.set_xticks(x)
    ax1.set_xticklabels(['Original\nStanley', 'Modified\nStanley', 'Baseline\nMPC', 'Latest\nMPC'])
    ax1.legend()
    ax1.grid(True, linestyle=':', alpha=0.6)
    
    # Add values on top of bars
    for rect in rects1 + rects2:
        h = rect.get_height()
        ax1.annotate(f'{h:.2f}m',
                     xy=(rect.get_x() + rect.get_width() / 2, h),
                     xytext=(0, 3), textcoords="offset points",
                     ha='center', va='bottom', fontsize=9)

    # Subplot 2: Speeds
    rects3 = ax2.bar(x - width/2, sharp_speeds, width, label='Sharp Curves Apex Speed', color='#e41a1c', alpha=0.85)
    rects4 = ax2.bar(x + width/2, smooth_speeds, width, label='Smooth Sections / Straight Speed', color='#377eb8', alpha=0.85)
    ax2.set_ylabel('Mean Speed [m/s]')
    ax2.set_title('Speed Retention: Sharp Apex vs Straights (Spielberg)', fontweight='bold')
    ax2.set_xticks(x)
    ax2.set_xticklabels(['Original\nStanley', 'Modified\nStanley', 'Baseline\nMPC', 'Latest\nMPC'])
    ax2.legend()
    ax2.grid(True, linestyle=':', alpha=0.6)
    
    for rect in rects3 + rects4:
        h = rect.get_height()
        ax2.annotate(f'{h:.1f}m/s',
                     xy=(rect.get_x() + rect.get_width() / 2, h),
                     xytext=(0, 3), textcoords="offset points",
                     ha='center', va='bottom', fontsize=9)

    plt.tight_layout()
    out_file = os.path.join(PLOTS_DIR, 'sharp_vs_smooth_turn_performance.png')
    plt.savefig(out_file, dpi=300)
    plt.close()
    print(f"[PLOT] Generated: {out_file}")


def plot_generalization(summary):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6), dpi=300)
    gen_tracks = ['Catalunya', 'Spa']
    ctrl_order = ['Original_Stanley', 'Modified_Stanley', 'Baseline_MPC', 'Latest_MPC']
    
    for idx, track in enumerate(gen_tracks):
        ax = ax1 if idx == 0 else ax2
        res_t = {r['controller']: r for r in summary if r['track_name'] == track}
        
        times = []
        labels = []
        bar_colors = []
        
        for c in ctrl_order:
            if c in res_t:
                r = res_t[c]
                t = r['lap_time'] if r['completed'] else 0.0
                times.append(t)
                bar_colors.append(COLORS.get(c, '#333'))
                status = f"{t:.1f}s" if r['completed'] else f"CRASH ({r['completion_pct']}%)"
                labels.append(status)
        
        x = np.arange(len(ctrl_order))
        bars = ax.bar(x, times, color=bar_colors, width=0.55, alpha=0.9)
        ax.set_ylabel('Lap Time [seconds]')
        ax.set_title(f'Generalization on Unseen Track: {track}', fontweight='bold')
        ax.set_xticks(x)
        ax.set_xticklabels(['Original\nStanley', 'Modified\nStanley', 'Baseline\nMPC', 'Latest\nMPC'])
        ax.grid(True, linestyle=':', alpha=0.6)
        
        for bar, label, c in zip(bars, labels, ctrl_order):
            h = bar.get_height()
            if h > 0:
                ax.annotate(label,
                            xy=(bar.get_x() + bar.get_width() / 2, h),
                            xytext=(0, 3), textcoords="offset points",
                            ha='center', va='bottom', fontweight='bold', fontsize=9)
            else:
                ax.annotate(label,
                            xy=(bar.get_x() + bar.get_width() / 2, 5),
                            xytext=(0, 3), textcoords="offset points",
                            ha='center', va='bottom', color='red', fontweight='bold', fontsize=9)

    plt.tight_layout()
    out_file = os.path.join(PLOTS_DIR, 'generalization_unseen_circuits.png')
    plt.savefig(out_file, dpi=300)
    plt.close()
    print(f"[PLOT] Generated: {out_file}")


def main():
    summary, telem = load_data()
    plot_velocity_vs_distance_time(telem)
    plot_steering_angle_vs_time(telem)
    plot_crosstrack_error(telem)
    plot_sharp_vs_smooth_bars(summary)
    plot_generalization(summary)
    print("\nAll publication plots successfully generated in 'benchmark_plots/'!")


if __name__ == '__main__':
    main()
