#!/usr/bin/env python3
"""
Generate publication-quality parameter sensitivity plots for Experiment 1.2.
"""

import os
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

EXP_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(EXP_DIR, 'data', 'exp1_2_parameters')
PLOTS_DIR = os.path.join(EXP_DIR, 'plots')
RESULTS_JSON = os.path.join(DATA_DIR, 'parameter_sweep_results.json')

os.makedirs(PLOTS_DIR, exist_ok=True)

plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
matplotlib.rcParams['font.family'] = 'sans-serif'
matplotlib.rcParams['font.size'] = 11


def load_sweep_data():
    with open(RESULTS_JSON, 'r') as f:
        return json.load(f)


def generate_sweep_plots(data):
    # -------------------------------------------------------------
    # 1. Pure Pursuit: Lookahead Distance vs CT RMSE & Lap Time
    # -------------------------------------------------------------
    pp_data = [d for d in data if d['controller'] == 'Pure_Pursuit' and d['parameter'] == 'lookahead_distance']
    if pp_data:
        x_vals = [d['value'] for d in pp_data]
        ct_rmse = [d['cross_track_rmse_m'] for d in pp_data]
        lap_times = [d['mean_lap_s'] if d['mean_lap_s'] is not None else np.nan for d in pp_data]

        fig, ax1 = plt.subplots(figsize=(8, 5), dpi=300)
        color = '#1976D2'
        ax1.set_xlabel('Lookahead Distance $L_d$ (m)', fontweight='bold')
        ax1.set_ylabel('Cross-Track RMSE (m)', color=color, fontweight='bold')
        line1 = ax1.plot(x_vals, ct_rmse, 'o-', color=color, linewidth=2.5, markersize=8, label='Cross-Track RMSE')
        ax1.tick_params(axis='y', labelcolor=color)

        ax2 = ax1.twinx()
        color = '#D32F2F'
        ax2.set_ylabel('Mean Lap Time (s)', color=color, fontweight='bold')
        line2 = ax2.plot(x_vals, lap_times, 's--', color=color, linewidth=2.5, markersize=8, label='Lap Time')
        ax2.tick_params(axis='y', labelcolor=color)

        lines = line1 + line2
        labels = [l.get_label() for l in lines]
        ax1.legend(lines, labels, loc='center right', frameon=True, facecolor='white', framealpha=0.9)
        plt.title('Pure Pursuit: Lookahead Distance Sensitivity ($L_d$)', fontsize=13, fontweight='bold', pad=15)
        fig.tight_layout()
        p = os.path.join(PLOTS_DIR, 'fig6_pure_pursuit_sensitivity.png')
        fig.savefig(p)
        plt.close(fig)
        print(f"Saved: {p}")

    # -------------------------------------------------------------
    # 2. Stanley: Gain (k) and Soft Damping (k_soft)
    # -------------------------------------------------------------
    stanley_k = [d for d in data if d['controller'] == 'Stanley_Controller' and d['parameter'] == 'gain_k']
    stanley_ks = [d for d in data if d['controller'] == 'Stanley_Controller' and d['parameter'] == 'software_k']
    
    if stanley_k and stanley_ks:
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5), dpi=300)
        
        # Gain k
        x_k = [d['value'] for d in stanley_k]
        rmse_k = [d['cross_track_rmse_m'] for d in stanley_k]
        rev_k = [d['steer_reversals'] for d in stanley_k]
        
        color = '#388E3C'
        ax1.plot(x_k, rmse_k, 'o-', color=color, linewidth=2.5, markersize=7, label='CT RMSE (m)')
        ax1.set_xlabel('Cross-Track Gain $k$', fontweight='bold')
        ax1.set_ylabel('Cross-Track RMSE (m)', color=color, fontweight='bold')
        ax1.tick_params(axis='y', labelcolor=color)
        
        ax1_twin = ax1.twinx()
        color2 = '#E64A19'
        ax1_twin.plot(x_k, rev_k, '^--', color=color2, linewidth=2.0, markersize=7, label='Steer Reversals')
        ax1_twin.set_ylabel('Steering Sign Reversals (Chatter)', color=color2, fontweight='bold')
        ax1_twin.tick_params(axis='y', labelcolor=color2)
        ax1.set_title('(a) Gain $k$ vs Tracking & Chatter', fontweight='bold')

        # Damping k_soft
        x_ks = [d['value'] for d in stanley_ks]
        rmse_ks = [d['cross_track_rmse_m'] for d in stanley_ks]
        rev_ks = [d['steer_reversals'] for d in stanley_ks]
        
        color = '#388E3C'
        ax2.plot(x_ks, rmse_ks, 'o-', color=color, linewidth=2.5, markersize=7, label='CT RMSE (m)')
        ax2.set_xlabel('Soft Damping $k_{soft}$ (m/s)', fontweight='bold')
        ax2.set_ylabel('Cross-Track RMSE (m)', color=color, fontweight='bold')
        ax2.tick_params(axis='y', labelcolor=color)

        ax2_twin = ax2.twinx()
        color2 = '#E64A19'
        ax2_twin.plot(x_ks, rev_ks, '^--', color=color2, linewidth=2.0, markersize=7, label='Steer Reversals')
        ax2_twin.set_ylabel('Steering Sign Reversals (Chatter)', color=color2, fontweight='bold')
        ax2_twin.tick_params(axis='y', labelcolor=color2)
        ax2.set_title('(b) Damping $k_{soft}$ vs Stability', fontweight='bold')

        fig.suptitle('Stanley Controller Parameter Sensitivity', fontsize=14, fontweight='bold', y=1.02)
        fig.tight_layout()
        p = os.path.join(PLOTS_DIR, 'fig7_stanley_sensitivity.png')
        fig.savefig(p)
        plt.close(fig)
        print(f"Saved: {p}")

    # -------------------------------------------------------------
    # 3. MPC: Horizon (N) and Slew Rate Penalty (w_ddelta)
    # -------------------------------------------------------------
    mpc_n = [d for d in data if d['controller'] == 'MPC_Controller' and d['parameter'] == 'horizon_N']
    mpc_w = [d for d in data if d['controller'] == 'MPC_Controller' and d['parameter'] == 'w_ddelta']
    
    if mpc_n and mpc_w:
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5), dpi=300)
        
        # Horizon N
        x_n = [d['value'] for d in mpc_n]
        rmse_n = [d['cross_track_rmse_m'] for d in mpc_n]
        time_n = [d['mean_lap_s'] for d in mpc_n]
        
        color = '#7B1FA2'
        ax1.plot(x_n, rmse_n, 'o-', color=color, linewidth=2.5, markersize=7, label='CT RMSE (m)')
        ax1.set_xlabel('Prediction Horizon Steps $N$', fontweight='bold')
        ax1.set_ylabel('Cross-Track RMSE (m)', color=color, fontweight='bold')
        ax1.tick_params(axis='y', labelcolor=color)
        
        ax1_twin = ax1.twinx()
        color2 = '#0288D1'
        ax1_twin.plot(x_n, time_n, 's--', color=color2, linewidth=2.0, markersize=7, label='Mean Lap (s)')
        ax1_twin.set_ylabel('Mean Lap Time (s)', color=color2, fontweight='bold')
        ax1_twin.tick_params(axis='y', labelcolor=color2)
        ax1.set_title('(a) Horizon $N$ vs Precision & Lap Time', fontweight='bold')

        # Slew rate weight w_ddelta
        x_w = [d['value'] for d in mpc_w]
        rmse_w = [d['cross_track_rmse_m'] for d in mpc_w]
        slew_w = [d['steer_rate_rms_radps'] for d in mpc_w]
        
        color = '#7B1FA2'
        ax2.plot(x_w, rmse_w, 'o-', color=color, linewidth=2.5, markersize=7, label='CT RMSE (m)')
        ax2.set_xlabel('Steering Slew Penalty $w_{\Delta \delta}$', fontweight='bold')
        ax2.set_ylabel('Cross-Track RMSE (m)', color=color, fontweight='bold')
        ax2.tick_params(axis='y', labelcolor=color)

        ax2_twin = ax2.twinx()
        color2 = '#C2185B'
        ax2_twin.plot(x_w, slew_w, 'd--', color=color2, linewidth=2.0, markersize=7, label='Steer RMS (rad/s)')
        ax2_twin.set_ylabel('Steering Rate RMS (rad/s)', color=color2, fontweight='bold')
        ax2_twin.tick_params(axis='y', labelcolor=color2)
        ax2.set_title('(b) Slew Penalty $w_{\Delta \delta}$ vs Actuator Stress', fontweight='bold')

        fig.suptitle('MPC Controller Parameter Sensitivity', fontsize=14, fontweight='bold', y=1.02)
        fig.tight_layout()
        p = os.path.join(PLOTS_DIR, 'fig8_mpc_sensitivity.png')
        fig.savefig(p)
        plt.close(fig)
        print(f"Saved: {p}")

    # -------------------------------------------------------------
    # 4. Follow-the-Gap: Bubble Radius
    # -------------------------------------------------------------
    ftg_b = [d for d in data if d['controller'] == 'Follow_The_Gap' and d['parameter'] == 'bubble_radius']
    if ftg_b:
        x_b = [d['value'] for d in ftg_b]
        laps_b = [d['mean_lap_s'] for d in ftg_b]
        min_lidar = [d['min_lidar_m'] for d in ftg_b]
        
        fig, ax1 = plt.subplots(figsize=(8, 5), dpi=300)
        color = '#FFA000'
        ax1.set_xlabel('Safety Bubble Radius $r_{bubble}$ (m)', fontweight='bold')
        ax1.set_ylabel('Mean Lap Time (s)', color=color, fontweight='bold')
        l1 = ax1.plot(x_b, laps_b, 'o-', color=color, linewidth=2.5, markersize=8, label='Lap Time (s)')
        ax1.tick_params(axis='y', labelcolor=color)

        ax2 = ax1.twinx()
        color2 = '#388E3C'
        ax2.set_ylabel('Min LiDAR Distance to Wall (m)', color=color2, fontweight='bold')
        l2 = ax2.plot(x_b, min_lidar, '^--', color=color2, linewidth=2.5, markersize=8, label='Min Clearance')
        ax2.tick_params(axis='y', labelcolor=color2)

        lines = l1 + l2
        labels = [l.get_label() for l in lines]
        ax1.legend(lines, labels, loc='center left', frameon=True, facecolor='white', framealpha=0.9)
        plt.title('Follow-the-Gap: Safety Bubble vs Clearance & Speed', fontsize=13, fontweight='bold', pad=15)
        fig.tight_layout()
        p = os.path.join(PLOTS_DIR, 'fig9_ftg_sensitivity.png')
        fig.savefig(p)
        plt.close(fig)
        print(f"Saved: {p}")

    # -------------------------------------------------------------
    # 5. Wall Following: Desired Distance & Kp
    # -------------------------------------------------------------
    wf_d = [d for d in data if d['controller'] == 'Wall_Following' and d['parameter'] == 'desired_distance']
    wf_kp = [d for d in data if d['controller'] == 'Wall_Following' and d['parameter'] == 'kp']
    
    if wf_d and wf_kp:
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5), dpi=300)
        
        # Desired distance
        x_d = [d['value'] for d in wf_d]
        time_d = [d['mean_lap_s'] for d in wf_d]
        rmse_d = [d['cross_track_rmse_m'] for d in wf_d]
        
        color = '#E64A19'
        ax1.plot(x_d, time_d, 'o-', color=color, linewidth=2.5, markersize=7, label='Lap Time (s)')
        ax1.set_xlabel('Desired Wall Distance $d_{des}$ (m)', fontweight='bold')
        ax1.set_ylabel('Mean Lap Time (s)', color=color, fontweight='bold')
        ax1.tick_params(axis='y', labelcolor=color)
        
        ax1_twin = ax1.twinx()
        color2 = '#5D4037'
        ax1_twin.plot(x_d, rmse_d, 's--', color=color2, linewidth=2.0, markersize=7, label='CT RMSE (m)')
        ax1_twin.set_ylabel('Cross-Track RMSE (m)', color=color2, fontweight='bold')
        ax1_twin.tick_params(axis='y', labelcolor=color2)
        ax1.set_title('(a) Desired Distance $d_{des}$ vs Lap Time', fontweight='bold')

        # Gain Kp
        x_kp = [d['value'] for d in wf_kp]
        slew_kp = [d['steer_rate_rms_radps'] for d in wf_kp]
        rev_kp = [d['steer_reversals'] for d in wf_kp]
        
        color = '#E64A19'
        ax2.plot(x_kp, slew_kp, 'o-', color=color, linewidth=2.5, markersize=7, label='Steer RMS (rad/s)')
        ax2.set_xlabel('Proportional Gain $K_p$', fontweight='bold')
        ax2.set_ylabel('Steering Rate RMS (rad/s)', color=color, fontweight='bold')
        ax2.tick_params(axis='y', labelcolor=color)

        ax2_twin = ax2.twinx()
        color2 = '#795548'
        ax2_twin.plot(x_kp, rev_kp, '^--', color=color2, linewidth=2.0, markersize=7, label='Reversals')
        ax2_twin.set_ylabel('Steering Reversals (Chatter)', color=color2, fontweight='bold')
        ax2_twin.tick_params(axis='y', labelcolor=color2)
        ax2.set_title('(b) Gain $K_p$ vs Chattering / Stability', fontweight='bold')

        fig.suptitle('Wall Follower Parameter Sensitivity', fontsize=14, fontweight='bold', y=1.02)
        fig.tight_layout()
        p = os.path.join(PLOTS_DIR, 'fig10_wall_follow_sensitivity.png')
        fig.savefig(p)
        plt.close(fig)
        print(f"Saved: {p}")


if __name__ == '__main__':
    data = load_sweep_data()
    generate_sweep_plots(data)
