#!/usr/bin/env python3
"""
Unifies all 40 benchmark evaluations (5 controllers x 8 tracks) into:
- `benchmark_5way_results.json`
- 7 publication-grade comparative figures in `benchmark_plots/`
"""

import os
import sys
import json
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

WS_ROOT = '/home/yeswanth/roboracer_ws'
PLOTS_DIR = os.path.join(WS_ROOT, 'benchmark_plots')
os.makedirs(PLOTS_DIR, exist_ok=True)

# 1. Merge datasets
with open(os.path.join(WS_ROOT, 'benchmark_results.json'), 'r') as f:
    base_results = json.load(f)

with open(os.path.join(WS_ROOT, 'benchmark_levine_baselines.json'), 'r') as f:
    levine_baselines = json.load(f)

with open(os.path.join(WS_ROOT, 'benchmark_hybrid_results.json'), 'r') as f:
    hybrid_results = json.load(f)

unified_results = base_results + levine_baselines + hybrid_results
with open(os.path.join(WS_ROOT, 'benchmark_5way_results.json'), 'w') as f:
    json.dump(unified_results, f, indent=2)

print(f"Unified dataset contains {len(unified_results)} benchmark runs across 5 controllers.")

# Load telemetry
with open(os.path.join(WS_ROOT, 'benchmark_telemetry.json'), 'r') as f:
    base_telem = json.load(f)

with open(os.path.join(WS_ROOT, 'benchmark_hybrid_telemetry.json'), 'r') as f:
    hybrid_telem = json.load(f)

# 2. Color Palette & Labels
COLORS = {
    'Original_Stanley': '#e66101',   # Burnt Orange
    'Modified_Stanley': '#5e3c99',   # Deep Purple
    'Baseline_MPC':     '#d01c8b',   # Magenta / Rose
    'Latest_MPC':       '#4dac26',   # Bright Green
    'Hybrid_Controller':'#0571b0'    # Deep Racing Blue
}

LABELS = {
    'Original_Stanley': '1. Normal Stanley (Classic)',
    'Modified_Stanley': '2. Upgraded Stanley (Feedforward + Pre-Braking)',
    'Baseline_MPC':     '3. Normal MPC (Baseline OSQP LTV)',
    'Latest_MPC':       '4. Upgraded MPC (Curv-Aware + Pre-Braking)',
    'Hybrid_Controller':'5. Hybrid Controller (ARM-HC Multi-Regime)'
}

plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.size'] = 11

# -------------------------------------------------------------
# Plot 1: Comprehensive 5-Way Controller Dashboard
# -------------------------------------------------------------
fig = plt.figure(figsize=(18, 12), dpi=300)
gs = gridspec.GridSpec(2, 2, figure=fig, hspace=0.32, wspace=0.22)

circuits_primary = ['Spielberg', 'Austin', 'Monza', 'BrandsHatch', 'Silverstone', 'Catalunya', 'Spa', 'Levine']
ctrl_keys = ['Original_Stanley', 'Modified_Stanley', 'Baseline_MPC', 'Latest_MPC', 'Hybrid_Controller']

# Lookup helper
def get_entry(track, ctrl):
    for r in unified_results:
        if r['track_name'].lower() == track.lower() and r['controller'] == ctrl:
            return r
    return None

# Subplot 1: Lap Times
ax1 = fig.add_subplot(gs[0, 0])
x_indices = np.arange(len(circuits_primary))
width = 0.15

for i, ctrl in enumerate(ctrl_keys):
    laps = []
    for trk in circuits_primary:
        e = get_entry(trk, ctrl)
        if e and e['completed'] and e['lap_time']:
            laps.append(e['lap_time'])
        else:
            laps.append(0.0) # DNF
    bars = ax1.bar(x_indices + (i - 2) * width, laps, width, label=LABELS[ctrl], color=COLORS[ctrl], alpha=0.9)
    for b, l in zip(bars, laps):
        if l == 0.0:
            ax1.text(b.get_x() + b.get_width()/2., 4, 'DNF', ha='center', va='bottom', fontsize=8, color='red', fontweight='bold', rotation=90)

ax1.set_title('Lap Time Comparison Across All Maps [s] (Lower is Better)', fontweight='bold')
ax1.set_xticks(x_indices)
ax1.set_xticklabels(circuits_primary, rotation=18)
ax1.set_ylabel('Lap Time [seconds]')
ax1.grid(True, linestyle=':', alpha=0.6)
ax1.legend(loc='upper right', fontsize=8)

# Subplot 2: Average Speed
ax2 = fig.add_subplot(gs[0, 1])
for i, ctrl in enumerate(ctrl_keys):
    speeds = []
    for trk in circuits_primary:
        e = get_entry(trk, ctrl)
        speeds.append(e['avg_speed'] if e else 0.0)
    ax2.bar(x_indices + (i - 2) * width, speeds, width, label=LABELS[ctrl], color=COLORS[ctrl], alpha=0.9)

ax2.set_title('Average Lap Speed Across All Maps [m/s] (Higher is Better)', fontweight='bold')
ax2.set_xticks(x_indices)
ax2.set_xticklabels(circuits_primary, rotation=18)
ax2.set_ylabel('Average Velocity [m/s]')
ax2.grid(True, linestyle=':', alpha=0.6)

# Subplot 3: Steering Rate Jitter
ax3 = fig.add_subplot(gs[1, 0])
for i, ctrl in enumerate(ctrl_keys):
    jitters = []
    for trk in circuits_primary:
        e = get_entry(trk, ctrl)
        jitters.append(e['steer_jitter_std'] if e else 0.0)
    ax3.bar(x_indices + (i - 2) * width, jitters, width, label=LABELS[ctrl], color=COLORS[ctrl], alpha=0.9)

ax3.set_title('Steering Chatter & Rate Jitter: std(δ_dot) [rad/s] (Lower is Better)', fontweight='bold')
ax3.set_xticks(x_indices)
ax3.set_xticklabels(circuits_primary, rotation=18)
ax3.set_ylabel('Steering Rate Std Dev [rad/s]')
ax3.axhline(0.12, color='red', linestyle='--', label='Jitter Ceiling (0.12 rad/s)')
ax3.grid(True, linestyle=':', alpha=0.6)
ax3.legend(loc='upper right', fontsize=8)

# Subplot 4: Track Completion Rate & Collisions
ax4 = fig.add_subplot(gs[1, 1])
comp_rates = []
col_counts = []
for ctrl in ctrl_keys:
    tot = len(circuits_primary)
    comps = sum(1 for trk in circuits_primary if get_entry(trk, ctrl) and get_entry(trk, ctrl)['completed'])
    crashes = sum(1 for trk in circuits_primary if get_entry(trk, ctrl) and get_entry(trk, ctrl)['crashed'])
    comp_rates.append((comps / tot) * 100.0)
    col_counts.append(crashes)

ctrl_x = np.arange(len(ctrl_keys))
bars4 = ax4.bar(ctrl_x, comp_rates, color=[COLORS[c] for c in ctrl_keys], width=0.55, alpha=0.9)
ax4.set_title('Total Track Completion Rate Across All 8 Maps [%]', fontweight='bold')
ax4.set_xticks(ctrl_x)
ax4.set_xticklabels(['Normal\nStanley', 'Upgraded\nStanley', 'Normal\nMPC', 'Upgraded\nMPC', 'Hybrid\n(ARM-HC)'])
ax4.set_ylabel('Completion Rate [%]')
ax4.set_ylim(0, 115)
for b, rate, crash in zip(bars4, comp_rates, col_counts):
    ax4.text(b.get_x() + b.get_width()/2., rate + 2.0, f"{rate:.1f}%\n({crash} crashes)", ha='center', va='bottom', fontsize=9, fontweight='bold')
ax4.grid(True, linestyle=':', alpha=0.6)

fig.suptitle('SYSTEMATIC 5-WAY CONTROLLER BENCHMARK EVALUATION ACROSS ALL F1TENTH MAPS\nNormal Stanley vs Upgraded Stanley vs Normal MPC vs Upgraded MPC vs Hybrid Controller', fontsize=15, fontweight='bold')
fig.savefig(os.path.join(PLOTS_DIR, 'comprehensive_5way_controller_dashboard.png'), bbox_inches='tight')
plt.close(fig)

# -------------------------------------------------------------
# Plot 2: Velocity vs Distance & Time (5-way comparison on Spielberg)
# -------------------------------------------------------------
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 9), dpi=300)
track = 'Spielberg'

# Add 4 baselines from base_telem
for ctrl in ['Original_Stanley', 'Modified_Stanley', 'Baseline_MPC', 'Latest_MPC']:
    if track in base_telem and ctrl in base_telem[track]:
        d = base_telem[track][ctrl]
        ax1.plot(d['distance'], d['speed'], label=LABELS[ctrl], color=COLORS[ctrl], linewidth=1.8, alpha=0.85)
        ax2.plot(d['time'], d['speed'], label=LABELS[ctrl], color=COLORS[ctrl], linewidth=1.8, alpha=0.85)

# Add Hybrid Controller from hybrid_telem
if track in hybrid_telem:
    hd = hybrid_telem[track]
    # Compute distance along trajectory
    h_dist = [0.0]
    for k in range(1, len(hd['x'])):
        h_dist.append(h_dist[-1] + math.hypot(hd['x'][k] - hd['x'][k-1], hd['y'][k] - hd['y'][k-1]))
    ax1.plot(h_dist, hd['speed'], label=LABELS['Hybrid_Controller'], color=COLORS['Hybrid_Controller'], linewidth=2.4)
    ax2.plot(hd['time'], hd['speed'], label=LABELS['Hybrid_Controller'], color=COLORS['Hybrid_Controller'], linewidth=2.4)

ax1.set_title(f'Velocity Profile vs Arc-Length Track Distance — {track} Circuit (5-Way Comparison)', fontweight='bold')
ax1.set_xlabel('Distance along track [m]')
ax1.set_ylabel('Velocity [m/s]')
ax1.set_xlim(0, 340)
ax1.set_ylim(0, 8.2)
ax1.axhline(7.5, color='#4dac26', linestyle='--', alpha=0.5, label='Speed Ceiling (7.5 m/s)')
ax1.legend(loc='lower right', fontsize=9, framealpha=0.95)
ax1.grid(True, linestyle=':', alpha=0.6)

ax2.set_title(f'Velocity vs Elapsed Simulation Time — Lap Duration Comparison ({track})', fontweight='bold')
ax2.set_xlabel('Elapsed Time [s]')
ax2.set_ylabel('Velocity [m/s]')
ax2.set_ylim(0, 8.2)
ax2.legend(loc='upper right', fontsize=9, framealpha=0.95)
ax2.grid(True, linestyle=':', alpha=0.6)

fig.tight_layout()
fig.savefig(os.path.join(PLOTS_DIR, 'velocity_vs_distance_time_5way.png'), bbox_inches='tight')
plt.close(fig)

# -------------------------------------------------------------
# Plot 3: Steering Angle Dynamics & Chatter Comparison
# -------------------------------------------------------------
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 9), dpi=300)

for ctrl in ['Original_Stanley', 'Modified_Stanley', 'Baseline_MPC', 'Latest_MPC']:
    if track in base_telem and ctrl in base_telem[track]:
        d = base_telem[track][ctrl]
        ax1.plot(d['time'], np.degrees(d['steer']), label=LABELS[ctrl], color=COLORS[ctrl], linewidth=1.6, alpha=0.8)
        ax2.plot(d['time'], d['steer_rate'], label=LABELS[ctrl], color=COLORS[ctrl], linewidth=1.4, alpha=0.75)

if track in hybrid_telem:
    hd = hybrid_telem[track]
    h_srate = [0.0]
    for k in range(1, len(hd['steer'])):
        dt_k = hd['time'][k] - hd['time'][k-1]
        h_srate.append(abs(hd['steer'][k] - hd['steer'][k-1]) / max(1e-3, dt_k))
    ax1.plot(hd['time'], np.degrees(hd['steer']), label=LABELS['Hybrid_Controller'], color=COLORS['Hybrid_Controller'], linewidth=2.2)
    ax2.plot(hd['time'], h_srate, label=LABELS['Hybrid_Controller'], color=COLORS['Hybrid_Controller'], linewidth=1.8)

ax1.set_title(f'Steering Angle δ(t) Dynamics — {track} (Turn 1 into Hairpin Entry)', fontweight='bold')
ax1.set_xlabel('Elapsed Time [s]')
ax1.set_ylabel('Steering Angle [degrees]')
ax1.set_xlim(0, 35)
ax1.legend(loc='upper right', fontsize=9, framealpha=0.95)
ax1.grid(True, linestyle=':', alpha=0.6)

ax2.set_title(f'Steering Slew Rate |δ_dot(t)| [rad/s] — Micro-Chatter & Actuator Jiggle Diagnosis', fontweight='bold')
ax2.set_xlabel('Elapsed Time [s]')
ax2.set_ylabel('Steering Rate [rad/s]')
ax2.set_xlim(0, 35)
ax2.set_ylim(0, 3.5)
ax2.legend(loc='upper right', fontsize=9, framealpha=0.95)
ax2.grid(True, linestyle=':', alpha=0.6)

fig.tight_layout()
fig.savefig(os.path.join(PLOTS_DIR, 'steering_angle_vs_time_5way.png'), bbox_inches='tight')
plt.close(fig)

# -------------------------------------------------------------
# Plot 4: Wall Jitter in Narrow Corridors (Levine Focus)
# -------------------------------------------------------------
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6), dpi=300)

levine_jitters = []
levine_times = []
for c in ctrl_keys:
    e = get_entry('Levine', c)
    levine_jitters.append(e['steer_jitter_std'] if e else 0.0)
    levine_times.append(e['lap_time'] if e and e['completed'] else 0.0)

b_j = ax1.bar(ctrl_x, levine_jitters, color=[COLORS[c] for c in ctrl_keys], width=0.55, alpha=0.9)
ax1.set_title('Steering Chatter in Narrow Corridors — Levine Circuit [rad/s]', fontweight='bold')
ax1.set_xticks(ctrl_x)
ax1.set_xticklabels(['Normal\nStanley', 'Upgraded\nStanley', 'Normal\nMPC', 'Upgraded\nMPC', 'Hybrid\n(ARM-HC)'])
ax1.set_ylabel('Steering Rate Std Dev [rad/s]')
ax1.axhline(0.12, color='red', linestyle='--', label='Chatter Threshold (0.12 rad/s)')
for b, j in zip(b_j, levine_jitters):
    ax1.text(b.get_x() + b.get_width()/2., j + 0.02, f"{j:.3f}\nrad/s", ha='center', va='bottom', fontsize=9, fontweight='bold')
ax1.legend(loc='upper right')
ax1.grid(True, linestyle=':', alpha=0.6)

b_t = ax2.bar(ctrl_x, levine_times, color=[COLORS[c] for c in ctrl_keys], width=0.55, alpha=0.9)
ax2.set_title('Lap Time on Compact Indoor Track — Levine Circuit [s]', fontweight='bold')
ax2.set_xticks(ctrl_x)
ax2.set_xticklabels(['Normal\nStanley', 'Upgraded\nStanley', 'Normal\nMPC', 'Upgraded\nMPC', 'Hybrid\n(ARM-HC)'])
ax2.set_ylabel('Lap Time [s]')
for b, t in zip(b_t, levine_times):
    ax2.text(b.get_x() + b.get_width()/2., t + 0.5, f"{t:.2f}s", ha='center', va='bottom', fontsize=9, fontweight='bold')
ax2.grid(True, linestyle=':', alpha=0.6)

fig.suptitle('NARROW CORRIDOR JITTER & LAP TIME PERFORMANCE — LEVINE INDOOR CIRCUIT', fontsize=14, fontweight='bold')
fig.tight_layout()
fig.savefig(os.path.join(PLOTS_DIR, 'wall_jitter_narrow_corridors_comparison.png'), bbox_inches='tight')
plt.close(fig)

print("[SUCCESS] All 5-Way Comparative Plots Generated Successfully!")
