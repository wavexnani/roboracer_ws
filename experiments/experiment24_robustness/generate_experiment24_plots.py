#!/usr/bin/env python3
"""
Generates publication-quality figures for Experiment 24:
1. robustness_vs_position_noise.png
2. robustness_vs_heading_noise.png
3. robustness_vs_delay.png
4. robustness_vs_slew_rate.png
5. robustness_combined_stress.png
"""

import os
import json
import numpy as np
import matplotlib.pyplot as plt

plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
plt.rcParams['font.family'] = 'DejaVu Sans'
plt.rcParams['font.size'] = 10
plt.rcParams['axes.titlesize'] = 11
plt.rcParams['axes.labelsize'] = 10
plt.rcParams['xtick.labelsize'] = 9
plt.rcParams['ytick.labelsize'] = 9
plt.rcParams['legend.fontsize'] = 9
plt.rcParams['figure.titlesize'] = 13

COLOR_RAW = '#d62728'       # Crimson Red
COLOR_HOLD16 = '#ff7f0e'    # Amber Orange
COLOR_LATTICE = '#1f77b4'   # Royal Blue
PALETTE = {'RAW': COLOR_RAW, 'HOLD16': COLOR_HOLD16, 'HOLD_LATTICE16': COLOR_LATTICE}

OUT_DIR = '/home/yeswanth/roboracer_ws/experiments/experiment24_robustness'
JSON_PATH = os.path.join(OUT_DIR, 'experiment24_results.json')

def load_data():
    if not os.path.exists(JSON_PATH):
        print(f"Results JSON not found at {JSON_PATH}")
        return None
    with open(JSON_PATH) as f:
        return json.load(f)

def plot_position_noise(data):
    p2 = [r for r in data if r['phase'] == 'Phase_2_PosNoise']
    if not p2:
        return
        
    sigmas = sorted(list(set(r['params']['sigma_pos_m'] for r in p2)))
    sigmas_cm = [s * 100.0 for s in sigmas]
    
    fig, axs = plt.subplots(2, 2, figsize=(12, 9))
    fig.suptitle('Experiment 24 — Robustness vs Perception Position Noise (Phase 2)', fontweight='bold')
    
    policies = ['RAW', 'HOLD16', 'HOLD_LATTICE16']
    
    # 1. Lateral RMSE
    ax = axs[0, 0]
    for pol in policies:
        means = []
        stds = []
        for s in sigmas:
            vals = [r['tracking']['lat_rmse_100hz_cm'] for r in p2 if r['policy'] == pol and r['params']['sigma_pos_m'] == s]
            means.append(np.mean(vals))
            stds.append(np.std(vals))
        ax.errorbar(sigmas_cm, means, yerr=stds, label=pol, color=PALETTE[pol], marker='o', capsize=4, linewidth=2)
    ax.set_xlabel(r'Position Noise $\sigma_{\mathrm{pos}}$ [cm]')
    ax.set_ylabel('100-Hz Lateral RMSE [cm]')
    ax.set_title('Tracking Accuracy vs Position Uncertainty')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='upper left')
    
    # 2. Relay Reversals
    ax = axs[0, 1]
    for pol in policies:
        means = []
        stds = []
        for s in sigmas:
            vals = [r['chatter']['relay_sign_reversals'] for r in p2 if r['policy'] == pol and r['params']['sigma_pos_m'] == s]
            means.append(np.mean(vals))
            stds.append(np.std(vals))
        ax.errorbar(sigmas_cm, means, yerr=stds, label=pol, color=PALETTE[pol], marker='s', capsize=4, linewidth=2)
    ax.set_xlabel(r'Position Noise $\sigma_{\mathrm{pos}}$ [cm]')
    ax.set_ylabel('Total Relay Sign Reversals (3 Laps)')
    ax.set_title('Actuator Chattering vs Position Uncertainty')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='center right')
    
    # 3. Steering Rate RMS
    ax = axs[1, 0]
    for pol in policies:
        means = []
        stds = []
        for s in sigmas:
            vals = [r['steering']['steer_rate_rms_rad_s'] for r in p2 if r['policy'] == pol and r['params']['sigma_pos_m'] == s]
            means.append(np.mean(vals))
            stds.append(np.std(vals))
        ax.errorbar(sigmas_cm, means, yerr=stds, label=pol, color=PALETTE[pol], marker='^', capsize=4, linewidth=2)
    ax.set_xlabel(r'Position Noise $\sigma_{\mathrm{pos}}$ [cm]')
    ax.set_ylabel('Actuator Slew Rate RMS [rad/s]')
    ax.set_title('Physical Steering Effort vs Position Uncertainty')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='center right')
    
    # 4. Phase-Aligned Error (Verified End-of-Actuation Window)
    ax = axs[1, 1]
    for pol in policies:
        means = []
        stds = []
        for s in sigmas:
            vals = [r['steering']['phase_err_end_rms_mrad'] for r in p2 if r['policy'] == pol and r['params']['sigma_pos_m'] == s]
            means.append(np.mean(vals))
            stds.append(np.std(vals))
        ax.errorbar(sigmas_cm, means, yerr=stds, label=pol, color=PALETTE[pol], marker='d', capsize=4, linewidth=2)
    ax.set_xlabel(r'Position Noise $\sigma_{\mathrm{pos}}$ [cm]')
    ax.set_ylabel(r'Verified Phase Error RMS $e_{\mathrm{end}}$ [mrad]')
    ax.set_title(r'Verified Phase Error vs Position Noise ($t_k + \tau + T_{\mathrm{ctrl}}$)')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='upper left')
    
    plt.tight_layout()
    out_file = os.path.join(OUT_DIR, 'robustness_vs_position_noise.png')
    plt.savefig(out_file, dpi=200)
    plt.close()
    print(f"Saved: {out_file}")

def plot_heading_noise(data):
    p3 = [r for r in data if r['phase'] == 'Phase_3_HeadNoise']
    if not p3:
        return
        
    sigmas_deg = sorted(list(set(r['params']['sigma_heading_deg'] for r in p3)))
    policies = ['RAW', 'HOLD16', 'HOLD_LATTICE16']
    
    fig, axs = plt.subplots(2, 2, figsize=(12, 9))
    fig.suptitle('Experiment 24 — Robustness vs Heading Noise (Phase 3)', fontweight='bold')
    
    # 1. Lateral RMSE
    ax = axs[0, 0]
    for pol in policies:
        means = []
        stds = []
        for s in sigmas_deg:
            vals = [r['tracking']['lat_rmse_100hz_cm'] for r in p3 if r['policy'] == pol and r['params']['sigma_heading_deg'] == s]
            means.append(np.mean(vals))
            stds.append(np.std(vals))
        ax.errorbar(sigmas_deg, means, yerr=stds, label=pol, color=PALETTE[pol], marker='o', capsize=4, linewidth=2)
    ax.set_xlabel(r'Heading Noise $\sigma_{\psi}$ [deg]')
    ax.set_ylabel('100-Hz Lateral RMSE [cm]')
    ax.set_title('Tracking Accuracy vs Heading Uncertainty')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='upper left')
    
    # 2. Relay Reversals
    ax = axs[0, 1]
    for pol in policies:
        means = []
        stds = []
        for s in sigmas_deg:
            vals = [r['chatter']['relay_sign_reversals'] for r in p3 if r['policy'] == pol and r['params']['sigma_heading_deg'] == s]
            means.append(np.mean(vals))
            stds.append(np.std(vals))
        ax.errorbar(sigmas_deg, means, yerr=stds, label=pol, color=PALETTE[pol], marker='s', capsize=4, linewidth=2)
    ax.set_xlabel(r'Heading Noise $\sigma_{\psi}$ [deg]')
    ax.set_ylabel('Total Relay Sign Reversals (3 Laps)')
    ax.set_title('Actuator Chattering vs Heading Uncertainty')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='center right')
    
    # 3. Steering Rate RMS
    ax = axs[1, 0]
    for pol in policies:
        means = []
        stds = []
        for s in sigmas_deg:
            vals = [r['steering']['steer_rate_rms_rad_s'] for r in p3 if r['policy'] == pol and r['params']['sigma_heading_deg'] == s]
            means.append(np.mean(vals))
            stds.append(np.std(vals))
        ax.errorbar(sigmas_deg, means, yerr=stds, label=pol, color=PALETTE[pol], marker='^', capsize=4, linewidth=2)
    ax.set_xlabel(r'Heading Noise $\sigma_{\psi}$ [deg]')
    ax.set_ylabel('Actuator Slew Rate RMS [rad/s]')
    ax.set_title('Physical Steering Effort vs Heading Uncertainty')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='center right')
    
    # 4. Phase-Aligned Error (Verified End-of-Actuation Window)
    ax = axs[1, 1]
    for pol in policies:
        means = []
        stds = []
        for s in sigmas_deg:
            vals = [r['steering']['phase_err_end_rms_mrad'] for r in p3 if r['policy'] == pol and r['params']['sigma_heading_deg'] == s]
            means.append(np.mean(vals))
            stds.append(np.std(vals))
        ax.errorbar(sigmas_deg, means, yerr=stds, label=pol, color=PALETTE[pol], marker='d', capsize=4, linewidth=2)
    ax.set_xlabel(r'Heading Noise $\sigma_{\psi}$ [deg]')
    ax.set_ylabel(r'Verified Phase Error RMS $e_{\mathrm{end}}$ [mrad]')
    ax.set_title(r'Verified Phase Error vs Heading Noise ($t_k + \tau + T_{\mathrm{ctrl}}$)')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='upper left')
    
    plt.tight_layout()
    out_file = os.path.join(OUT_DIR, 'robustness_vs_heading_noise.png')
    plt.savefig(out_file, dpi=200)
    plt.close()
    print(f"Saved: {out_file}")

def plot_delay_mismatch(data):
    p4 = [r for r in data if r['phase'] == 'Phase_4_DelayMismatch']
    if not p4:
        return
        
    delays_ms = sorted(list(set(r['params']['actual_delay_ms'] for r in p4)))
    policies = ['RAW', 'HOLD16', 'HOLD_LATTICE16']
    
    fig, axs = plt.subplots(2, 2, figsize=(12, 9))
    fig.suptitle('Experiment 24 — Robustness vs Actuator Delay Mismatch (Phase 4)', fontweight='bold')
    
    # 1. Lateral RMSE
    ax = axs[0, 0]
    for pol in policies:
        means = []
        for d in delays_ms:
            vals = [r['tracking']['lat_rmse_100hz_cm'] for r in p4 if r['policy'] == pol and r['params']['actual_delay_ms'] == d]
            means.append(np.mean(vals))
        ax.plot(delays_ms, means, label=pol, color=PALETTE[pol], marker='o', linewidth=2)
    ax.axvline(20, color='gray', linestyle=':', label='Controller Nominal (20 ms)')
    ax.set_xlabel('Actual Actuator Delay [ms]')
    ax.set_ylabel('Mean Lateral RMSE [cm] (4 Tracks)')
    ax.set_title('Tracking Accuracy vs Delay Mismatch')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='upper left')
    
    # 2. Relay Reversals
    ax = axs[0, 1]
    for pol in policies:
        means = []
        for d in delays_ms:
            vals = [r['chatter']['relay_sign_reversals'] for r in p4 if r['policy'] == pol and r['params']['actual_delay_ms'] == d]
            means.append(np.mean(vals))
        ax.plot(delays_ms, means, label=pol, color=PALETTE[pol], marker='s', linewidth=2)
    ax.axvline(20, color='gray', linestyle=':', label='Controller Nominal (20 ms)')
    ax.set_xlabel('Actual Actuator Delay [ms]')
    ax.set_ylabel('Total Relay Reversals (3 Laps)')
    ax.set_title('Actuator Chattering vs Delay Mismatch')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='center right')
    
    # 3. Phase Error 40ms vs Delay
    ax = axs[1, 0]
    for pol in policies:
        means = []
        for d in delays_ms:
            vals = [r['steering']['phase_err_40ms_rms_mrad'] for r in p4 if r['policy'] == pol and r['params']['actual_delay_ms'] == d]
            means.append(np.mean(vals))
        ax.plot(delays_ms, means, label=pol, color=PALETTE[pol], marker='^', linewidth=2)
    ax.axvline(20, color='gray', linestyle=':', label='Controller Nominal (20 ms)')
    ax.set_xlabel('Actual Actuator Delay [ms]')
    ax.set_ylabel(r'Phase Error RMS $e_{40\mathrm{ms}}$ [mrad]')
    ax.set_title('Cycle Boundary Phase Error vs Delay')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='upper left')
    
    # 4. Phase Error End-of-Window vs Delay
    ax = axs[1, 1]
    for pol in policies:
        means = []
        for d in delays_ms:
            vals = [r['steering']['phase_err_end_rms_mrad'] for r in p4 if r['policy'] == pol and r['params']['actual_delay_ms'] == d]
            means.append(np.mean(vals))
        ax.plot(delays_ms, means, label=pol, color=PALETTE[pol], marker='d', linewidth=2)
    ax.axvline(20, color='gray', linestyle=':', label='Controller Nominal (20 ms)')
    ax.set_xlabel('Actual Actuator Delay [ms]')
    ax.set_ylabel(r'Phase Error RMS $e_{\mathrm{end}}$ [mrad]')
    ax.set_title('End-of-Actuation Window Phase Error vs Delay')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='upper left')
    
    plt.tight_layout()
    out_file = os.path.join(OUT_DIR, 'robustness_vs_delay.png')
    plt.savefig(out_file, dpi=200)
    plt.close()
    print(f"Saved: {out_file}")

def plot_slew_rate_mismatch(data):
    p5 = [r for r in data if r['phase'] == 'Phase_5_SlewQuantumMismatch']
    if not p5:
        return
        
    slews = sorted(list(set(r['params']['actual_slew_rad_s'] for r in p5)))
    quantums_mrad = [s * 10.0 for s in slews]
    policies = ['RAW', 'HOLD16', 'HOLD_LATTICE16']
    
    fig, axs = plt.subplots(2, 2, figsize=(12, 9))
    fig.suptitle('Experiment 24 — Robustness vs Actuator Slew Rate & Relay Quantum (Phase 5)', fontweight='bold')
    
    # 1. Lateral RMSE
    ax = axs[0, 0]
    for pol in policies:
        means = []
        for s in slews:
            vals = [r['tracking']['lat_rmse_100hz_cm'] for r in p5 if r['policy'] == pol and r['params']['actual_slew_rad_s'] == s]
            means.append(np.mean(vals))
        ax.plot(slews, means, label=pol, color=PALETTE[pol], marker='o', linewidth=2)
    ax.axvline(3.2, color='gray', linestyle=':', label='Nominal 3.2 rad/s (q=32 mrad)')
    ax.set_xlabel('Actual Slew Rate $sv_{\mathrm{max}}$ [rad/s] (Top: $q=sv_{\mathrm{max}}\cdot 10\mathrm{ms}$)')
    ax.set_ylabel('Mean Lateral RMSE [cm] (4 Tracks)')
    ax.set_title('Tracking Accuracy vs Slew Rate Mismatch')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='upper left')
    
    # 2. Relay Reversals
    ax = axs[0, 1]
    for pol in policies:
        means = []
        for s in slews:
            vals = [r['chatter']['relay_sign_reversals'] for r in p5 if r['policy'] == pol and r['params']['actual_slew_rad_s'] == s]
            means.append(np.mean(vals))
        ax.plot(slews, means, label=pol, color=PALETTE[pol], marker='s', linewidth=2)
    ax.axvline(3.2, color='gray', linestyle=':', label='Nominal 3.2 rad/s (q=32 mrad)')
    ax.set_xlabel('Actual Slew Rate $sv_{\mathrm{max}}$ [rad/s]')
    ax.set_ylabel('Total Relay Reversals (3 Laps)')
    ax.set_title('Actuator Chattering vs Slew Rate Mismatch')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='center right')
    
    # 3. Steering Rate RMS
    ax = axs[1, 0]
    for pol in policies:
        means = []
        for s in slews:
            vals = [r['steering']['steer_rate_rms_rad_s'] for r in p5 if r['policy'] == pol and r['params']['actual_slew_rad_s'] == s]
            means.append(np.mean(vals))
        ax.plot(slews, means, label=pol, color=PALETTE[pol], marker='^', linewidth=2)
    ax.axvline(3.2, color='gray', linestyle=':', label='Nominal 3.2 rad/s')
    ax.set_xlabel('Actual Slew Rate $sv_{\mathrm{max}}$ [rad/s]')
    ax.set_ylabel('Actuator Slew Rate RMS [rad/s]')
    ax.set_title('Physical Steering Rate RMS vs Actuator Limit')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='center right')
    
    # 4. Phase-Aligned Error (Verified End-of-Actuation Window)
    ax = axs[1, 1]
    for pol in policies:
        means = []
        for s in slews:
            vals = [r['steering']['phase_err_end_rms_mrad'] for r in p5 if r['policy'] == pol and r['params']['actual_slew_rad_s'] == s]
            means.append(np.mean(vals))
        ax.plot(slews, means, label=pol, color=PALETTE[pol], marker='d', linewidth=2)
    ax.axvline(3.2, color='gray', linestyle=':', label='Nominal 3.2 rad/s')
    ax.set_xlabel('Actual Slew Rate $sv_{\mathrm{max}}$ [rad/s] (Relay Quantum $q = sv_{\mathrm{max}} \cdot 10\mathrm{ms}$)')
    ax.set_ylabel(r'Verified Phase Error RMS $e_{\mathrm{end}}$ [mrad]')
    ax.set_title(r'Verified Phase Error vs Slew Rate ($t_k + \tau + T_{\mathrm{ctrl}}$)')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='upper left')
    
    plt.tight_layout()
    out_file = os.path.join(OUT_DIR, 'robustness_vs_slew_rate.png')
    plt.savefig(out_file, dpi=200)
    plt.close()
    print(f"Saved: {out_file}")

def plot_timing_jitter(data):
    p6 = [r for r in data if r['phase'] == 'Phase_6_TimingJitter']
    p1_nom = [r for r in data if r['phase'] == 'Phase_1_Nominal' and r['track'] in ['Spielberg', 'Austin', 'Monza', 'Spa']]
    if not p6:
        return
        
    screening_tracks = ['Spielberg', 'Austin', 'Monza', 'Spa']
    policies = ['RAW', 'HOLD16', 'HOLD_LATTICE16']
    
    fig, axs = plt.subplots(2, 2, figsize=(13, 9))
    fig.suptitle('Experiment 24 — Robustness vs Timing Jitter (20/30 ms toggling, Phase 6)', fontweight='bold')
    
    # 1. Lateral RMSE: Nominal vs Jitter
    ax = axs[0, 0]
    bar_width = 0.25
    x = np.arange(len(screening_tracks))
    for idx, pol in enumerate(policies):
        means = []
        stds = []
        for trk in screening_tracks:
            vals = [r['tracking']['lat_rmse_100hz_cm'] for r in p6 if r['policy'] == pol and r['track'] == trk]
            means.append(np.mean(vals))
            stds.append(np.std(vals))
        ax.bar(x + idx * bar_width, means, bar_width, yerr=stds, label=f"{pol} (Jitter)", color=PALETTE[pol], capsize=3)
    # Overlay nominal points
    for idx, pol in enumerate(policies):
        nom_vals = [r['tracking']['lat_rmse_100hz_cm'] for trk in screening_tracks for r in p1_nom if r['policy'] == pol and r['track'] == trk]
        ax.scatter(x + idx * bar_width, nom_vals, color='black', marker='x', zorder=5, label='Nominal 20ms' if idx == 0 else "")
    ax.set_xticks(x + bar_width)
    ax.set_xticklabels(screening_tracks)
    ax.set_ylabel('100-Hz Lateral RMSE [cm]')
    ax.set_title('Tracking Accuracy: Nominal vs Timing Jitter')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='upper right')
    
    # 2. Relay Reversals: Jitter
    ax = axs[0, 1]
    for idx, pol in enumerate(policies):
        means = []
        stds = []
        for trk in screening_tracks:
            vals = [r['chatter']['relay_sign_reversals'] for r in p6 if r['policy'] == pol and r['track'] == trk]
            means.append(np.mean(vals))
            stds.append(np.std(vals))
        ax.bar(x + idx * bar_width, means, bar_width, yerr=stds, label=f"{pol} (Jitter)", color=PALETTE[pol], capsize=3)
    # Overlay nominal points
    for idx, pol in enumerate(policies):
        nom_vals = [r['chatter']['relay_sign_reversals'] for trk in screening_tracks for r in p1_nom if r['policy'] == pol and r['track'] == trk]
        ax.scatter(x + idx * bar_width, nom_vals, color='black', marker='x', zorder=5, label='Nominal 20ms' if idx == 0 else "")
    ax.set_xticks(x + bar_width)
    ax.set_xticklabels(screening_tracks)
    ax.set_ylabel('Total Relay Sign Reversals (3 Laps)')
    ax.set_title('Actuator Chattering under Timing Jitter')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='upper right')
    
    # 3. Slew Rate RMS
    ax = axs[1, 0]
    for idx, pol in enumerate(policies):
        means = []
        stds = []
        for trk in screening_tracks:
            vals = [r['steering']['steer_rate_rms_rad_s'] for r in p6 if r['policy'] == pol and r['track'] == trk]
            means.append(np.mean(vals))
            stds.append(np.std(vals))
        ax.bar(x + idx * bar_width, means, bar_width, yerr=stds, label=f"{pol}", color=PALETTE[pol], capsize=3)
    ax.set_xticks(x + bar_width)
    ax.set_xticklabels(screening_tracks)
    ax.set_ylabel('Actuator Slew Rate RMS [rad/s]')
    ax.set_title('Physical Steering Effort under Timing Jitter')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='upper right')
    
    # 4. Verified Phase Error RMS
    ax = axs[1, 1]
    for idx, pol in enumerate(policies):
        means = []
        stds = []
        for trk in screening_tracks:
            vals = [r['steering']['phase_err_end_rms_mrad'] for r in p6 if r['policy'] == pol and r['track'] == trk]
            means.append(np.mean(vals))
            stds.append(np.std(vals))
        ax.bar(x + idx * bar_width, means, bar_width, yerr=stds, label=f"{pol}", color=PALETTE[pol], capsize=3)
    ax.set_xticks(x + bar_width)
    ax.set_xticklabels(screening_tracks)
    ax.set_ylabel(r'Verified Phase Error RMS $e_{\mathrm{end}}$ [mrad]')
    ax.set_title('Phase Error under Timing Jitter')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='upper right')
    
    plt.tight_layout()
    out_file = os.path.join(OUT_DIR, 'robustness_vs_timing_jitter.png')
    plt.savefig(out_file, dpi=200)
    plt.close()
    print(f"Saved: {out_file}")

def plot_combined_stress(data):
    p7 = [r for r in data if r['phase'] == 'Phase_7_CombinedStress']
    if not p7:
        return
        
    stresses = ['STRESS_1', 'STRESS_2', 'STRESS_3']
    policies = ['RAW', 'HOLD16', 'HOLD_LATTICE16']
    
    fig, axs = plt.subplots(1, 3, figsize=(15, 5))
    fig.suptitle('Experiment 24 — Combined Stress Cases Across All 7 Circuits (Phase 7)', fontweight='bold')
    
    # 1. Lateral RMSE across stress cases
    ax = axs[0]
    bar_width = 0.25
    x = np.arange(len(stresses))
    for idx, pol in enumerate(policies):
        means = []
        stds = []
        for st in stresses:
            vals = [r['tracking']['lat_rmse_100hz_cm'] for r in p7 if r['policy'] == pol and r['condition'] == st]
            means.append(np.mean(vals))
            stds.append(np.std(vals))
        ax.bar(x + idx * bar_width, means, bar_width, yerr=stds, label=pol, color=PALETTE[pol], capsize=3)
    ax.set_xticks(x + bar_width)
    ax.set_xticklabels(['STRESS-1\n(Pos 1cm, Head 0.5°\nDelay 30ms, Slew 3.0)', 'STRESS-2\n(Pos 2cm, Head 1.0°\nDelay 40ms, Slew 2.8)', 'STRESS-3\n(Pos 1cm, Head 0.5°\nJitter 20/30, Slew 3.4)'])
    ax.set_ylabel('Mean Lateral RMSE [cm] (7 Tracks)')
    ax.set_title('Tracking Robustness under Compound Stress')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='upper left')
    
    # 2. Relay Reversals
    ax = axs[1]
    for idx, pol in enumerate(policies):
        means = []
        stds = []
        for st in stresses:
            vals = [r['chatter']['relay_sign_reversals'] for r in p7 if r['policy'] == pol and r['condition'] == st]
            means.append(np.mean(vals))
            stds.append(np.std(vals))
        ax.bar(x + idx * bar_width, means, bar_width, yerr=stds, label=pol, color=PALETTE[pol], capsize=3)
    ax.set_xticks(x + bar_width)
    ax.set_xticklabels(['STRESS-1', 'STRESS-2', 'STRESS-3'])
    ax.set_ylabel('Mean Relay Reversals (3 Laps)')
    ax.set_title('Chatter Suppression under Compound Stress')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='upper right')
    
    # 3. Fraction Held / Lattice
    ax = axs[2]
    for idx, pol in enumerate(policies):
        means = []
        for st in stresses:
            if pol == 'RAW':
                val = 0.0
            elif pol == 'HOLD16':
                val = np.mean([r['command_mod']['frac_held'] for r in p7 if r['policy'] == pol and r['condition'] == st])
            elif pol == 'HOLD_LATTICE16':
                val = np.mean([r['command_mod']['frac_lattice'] for r in p7 if r['policy'] == pol and r['condition'] == st])
            means.append(val * 100.0)
        ax.bar(x + idx * bar_width, means, bar_width, label=f"{pol} (% Active)", color=PALETTE[pol])
    ax.set_xticks(x + bar_width)
    ax.set_xticklabels(['STRESS-1', 'STRESS-2', 'STRESS-3'])
    ax.set_ylabel('% Cycles Modified')
    ax.set_title('Interface Policy Activity under Stress')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='upper right')
    
    plt.tight_layout()
    out_file = os.path.join(OUT_DIR, 'robustness_combined_stress.png')
    plt.savefig(out_file, dpi=200)
    plt.close()
    print(f"Saved: {out_file}")

def main():
    data = load_data()
    if data is None:
        return
    print(f"Loaded {len(data)} run records from experiment24_results.json.")
    plot_position_noise(data)
    plot_heading_noise(data)
    plot_delay_mismatch(data)
    plot_slew_rate_mismatch(data)
    plot_timing_jitter(data)
    plot_combined_stress(data)
    print("All Experiment 24 plots generated successfully!")

if __name__ == '__main__':
    main()
