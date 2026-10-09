#!/usr/bin/env python3
"""
Analysis helper for Experiment 23 results.
Extracts and formats all tables and statistical summaries required for README.md.
"""

import os
import json
import numpy as np

RESULTS_FILE = '/home/yeswanth/roboracer_ws/experiments/experiment23/experiment23_results.json'

with open(RESULTS_FILE, 'r') as f:
    data = json.load(f)

tracks = data['tracks']
policies = data['policies']
runs = data['runs']
det = data['determinism']

print("="*120)
print("1. CROSS-TRACK SUMMARY TABLE (OVERALL 3 LAPS)")
print("="*120)
hdr = (
    f"{'Track':<12} | {'Policy':<15} | {'100Hz LatRMSE':<13} | {'MeanSignLat':<11} | {'MaxLat':<7} | "
    f"{'100Hz Head':<10} | {'SteerRateRMS':<12} | {'Reversals':<9} | {'Persist≥4':<9} | "
    f"{'Streak':<6} | {'5-7Hz (r)':<9} | {'Held%':<6} | {'Lat%':<6} | {'LiDAR':<6} | {'Solve[ms]'}"
)
print(hdr)
print("-" * 150)

for t in tracks:
    for p in policies:
        r = runs[t][p]['overall']
        tr = r['tracking']
        st = r['steering']
        ch = r['chatter']
        fq = r['frequency']
        cm = r['command_mod']
        sf = runs[t][p]['safety']
        cp = runs[t][p]['compute']
        
        line = (
            f"{t:<12} | {p:<15} | "
            f"{tr['lat_rmse_100hz_cm']:7.3f} cm   | "
            f"{tr['lat_mean_signed_cm']:+6.2f} cm   | "
            f"{tr['lat_max_abs_cm']:5.2f}cm | "
            f"{tr['head_rmse_100hz_deg']:6.2f}°   | "
            f"{st['steer_rate_rms_rad_s']:6.3f} rad/s | "
            f"{ch['relay_sign_reversals']:<9d} | "
            f"{ch['persistent_cycles_ge4']:<9d} | "
            f"{ch['longest_alternating_streak']:<6d} | "
            f"{fq['energy_5_7_rad_s']:7.4f} | "
            f"{cm['frac_held']*100:4.1f}% | "
            f"{cm['frac_lattice']*100:4.1f}% | "
            f"{sf['min_lidar_m']:4.2f}m | "
            f"{cp['mean_solve_ms']:4.2f}ms"
        )
        print(line)
    print("-" * 150)

print("\n" + "="*120)
print("2. TRACKING BIAS AUDIT (SIGNED MEAN & MEDIAN LATERAL ERROR ACROSS 3 LAPS)")
print("="*120)
bias_hdr = f"{'Track':<12} | {'Policy':<15} | {'Mean Signed [cm]':<16} | {'Median Signed [cm]':<18} | {'Std Dev [cm]':<12} | {'RMSE [cm]':<10}"
print(bias_hdr)
print("-" * 90)
for t in tracks:
    for p in policies:
        tr = runs[t][p]['overall']['tracking']
        print(f"{t:<12} | {p:<15} | {tr['lat_mean_signed_cm']:+16.3f} | {tr['lat_median_signed_cm']:+18.3f} | {tr['lat_std_cm']:12.3f} | {tr['lat_rmse_100hz_cm']:10.3f}")
    print("-" * 90)

print("\n" + "="*120)
print("3. HOLD16 TRANSITION AUDIT (>32 mrad INCREMENTS)")
print("="*120)
total_gt_32 = 0
audit_classes = {}
for t in tracks:
    aud = runs[t]['HOLD16']['transition_audit']
    cnt = aud['total_gt_32mrad_events']
    total_gt_32 += cnt
    print(f"Track {t:<12}: {cnt:3d} events > 32 mrad")
    for cls_name, cls_data in aud['by_class'].items():
        if cls_name not in audit_classes:
            audit_classes[cls_name] = {'count': 0, 'max_inc': 0.0}
        audit_classes[cls_name]['count'] += cls_data['count']
        audit_classes[cls_name]['max_inc'] = max(audit_classes[cls_name]['max_inc'], cls_data['max_inc_mrad'])
        print(f"   Class '{cls_name}': count = {cls_data['count']:3d}, max_inc = {cls_data['max_inc_mrad']:.2f} mrad")

print(f"\nTOTAL HOLD16 > 32 mrad events across all 7 tracks: {total_gt_32}")
for cls_name, info in audit_classes.items():
    print(f"   Summary '{cls_name}': {info['count']} events, max increment = {info['max_inc']:.2f} mrad")

print("\n" + "="*120)
print("4. HOLD_LATTICE16 BUCKETED COMMAND MODIFICATION")
print("="*120)
# Aggregate bucketed analysis across all 7 tracks for HOLD_LATTICE16
agg_buckets = {
    'by_curvature': {'straight_gentle_lt_0_02': [], 'medium_curve_0_02_to_0_06': [], 'tight_corner_ge_0_06': []},
    'by_speed': {'low_speed_lt_3_5': [], 'medium_speed_3_5_to_5_5': [], 'high_speed_ge_5_5': []},
    'by_lateral_error': {'low_err_lt_2cm': [], 'medium_err_2cm_to_5cm': [], 'high_err_ge_5cm': []}
}

for t in tracks:
    b = runs[t]['HOLD_LATTICE16']['bucketed_command_mod']
    for cat in ['by_curvature', 'by_speed', 'by_lateral_error']:
        for bname, bstats in b[cat].items():
            if bstats['count'] > 0:
                agg_buckets[cat][bname].append(bstats)

for cat in ['by_curvature', 'by_speed', 'by_lateral_error']:
    print(f"\n--- Category: {cat} ---")
    print(f"{'Bucket':<28} | {'Total Samples':<13} | {'Frac Held':<10} | {'Frac Mod':<10} | {'RMS Mod [mrad]':<14} | {'Signed Mean [mrad]':<18} | {'Max Mod [mrad]'}")
    print("-" * 115)
    for bname, blist in agg_buckets[cat].items():
        tot_cnt = sum(x['count'] for x in blist)
        w_held = sum(x['frac_held'] * x['count'] for x in blist) / tot_cnt
        w_mod = sum(x['frac_changed'] * x['count'] for x in blist) / tot_cnt
        w_rms = np.sqrt(sum((x['rms_mod_mrad']**2) * x['count'] for x in blist) / tot_cnt)
        w_mean = sum(x['mean_signed_mrad'] * x['count'] for x in blist) / tot_cnt
        max_m = max(x['max_mod_mrad'] for x in blist)
        print(f"{bname:<28} | {tot_cnt:<13d} | {w_held*100:9.1f}% | {w_mod*100:9.1f}% | {w_rms:14.2f} | {w_mean:+18.2f} | {max_m:14.2f}")

print("\n" + "="*120)
print("5. LONG-TERM STABILITY (DRIFT ACROSS LAPS 1 -> 2 -> 3)")
print("="*120)
print(f"{'Track':<12} | {'Policy':<15} | {'Lap 1 LatRMSE':<13} | {'Lap 2 LatRMSE':<13} | {'Lap 3 LatRMSE':<13} | {'Lap 1 Time':<10} | {'Lap 2 Time':<10} | {'Lap 3 Time':<10}")
print("-" * 110)
for t in tracks:
    for p in policies:
        laps = runs[t][p]['per_lap']
        print(f"{t:<12} | {p:<15} | "
              f"{laps[0]['tracking']['lat_rmse_100hz_cm']:7.3f} cm    | "
              f"{laps[1]['tracking']['lat_rmse_100hz_cm']:7.3f} cm    | "
              f"{laps[2]['tracking']['lat_rmse_100hz_cm']:7.3f} cm    | "
              f"{laps[0]['lap_time_s']:7.2f}s   | "
              f"{laps[1]['lap_time_s']:7.2f}s   | "
              f"{laps[2]['lap_time_s']:7.2f}s")
    print("-" * 110)
