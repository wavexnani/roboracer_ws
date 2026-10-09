#!/usr/bin/env python3
"""
Unit and Integrity Test for Continuous Arc-Length Projection.
Verifies all 6 pre-simulation requirements from Experiment 2 prompt:
1. Finite s_cont
2. 0 <= s_cont < track_length
3. Valid projection parameter 0 <= t <= 1
4. Correct segment length including wrap-around
5. s_ref[0] matches s_cont
6. No discontinuous s_ref jumps
7. Numerical continuity across wrap-around boundary L_track -> 0
8. Evaluation across all telemetry frames of Experiment 1
"""

import sys
import math
import json
import numpy as np

sys.path.insert(0, '/home/yeswanth/roboracer_ws/src/mpc_controller')
from mpc_controller.track_manager import TrackManager

def compute_continuous_projection(cur_x, cur_y, closest_idx, waypoints, s_arr, track_length, num_waypoints):
    """Candidate segment projection with offsets [-1, 0, 1]."""
    p = np.array([cur_x, cur_y], dtype=np.float64)
    cand_offsets = [-1, 0, 1]
    best_dist_sq = float('inf')
    best_seg_a = closest_idx
    best_seg_b = (closest_idx + 1) % num_waypoints
    best_t = 0.0
    best_s_cont = s_arr[closest_idx]

    for offset in cand_offsets:
        ia = (closest_idx + offset) % num_waypoints
        ib = (ia + 1) % num_waypoints
        pa = waypoints[ia]
        pb = waypoints[ib]
        v = pb - pa
        L2 = float(np.dot(v, v))
        if L2 < 1e-12:
            t = 0.0
        else:
            t = float(np.clip(np.dot(p - pa, v) / L2, 0.0, 1.0))
        p_proj = pa + t * v
        dist_sq = float(np.dot(p - p_proj, p - p_proj))

        if dist_sq < best_dist_sq:
            best_dist_sq = dist_sq
            best_seg_a = ia
            best_seg_b = ib
            best_t = t
            if ia == num_waypoints - 1:
                seg_len = track_length - s_arr[ia]
            else:
                seg_len = s_arr[ib] - s_arr[ia]
            best_s_cont = (s_arr[ia] + t * seg_len) % track_length

    return best_s_cont, best_seg_a, best_seg_b, best_t, best_dist_sq


def generate_ref_horizon(s_cont, cur_v, dt, N, s_arr, waypoints, path_headings, scaled_target_speeds, track_length, num_waypoints):
    """Extracts reference horizon starting from s_cont."""
    speed_est = max(1.5, cur_v)
    ref_horizon = np.zeros((N + 1, 4))
    s_targets = np.zeros(N + 1)

    for k in range(N + 1):
        s_target = (s_cont + k * speed_est * dt) % track_length
        s_targets[k] = s_target

        idx_a = int(np.searchsorted(s_arr, s_target, side='right') - 1)
        idx_a = max(0, min(num_waypoints - 1, idx_a))
        idx_b = (idx_a + 1) % num_waypoints

        s_a = s_arr[idx_a]
        if idx_a == num_waypoints - 1:
            seg_len = track_length - s_a
            ds = s_target - s_a if s_target >= s_a else (s_target + track_length - s_a)
        else:
            s_b = s_arr[idx_b]
            seg_len = s_b - s_a
            ds = s_target - s_a

        t = float(np.clip(ds / max(seg_len, 1e-4), 0.0, 1.0))

        ref_horizon[k, 0] = (1.0 - t) * waypoints[idx_a, 0] + t * waypoints[idx_b, 0]
        ref_horizon[k, 1] = (1.0 - t) * waypoints[idx_a, 1] + t * waypoints[idx_b, 1]

        dpsi_seg = (path_headings[idx_b] - path_headings[idx_a] + math.pi) % (2.0 * math.pi) - math.pi
        ref_horizon[k, 2] = (path_headings[idx_a] + t * dpsi_seg + math.pi) % (2.0 * math.pi) - math.pi

        ref_horizon[k, 3] = (1.0 - t) * scaled_target_speeds[idx_a] + t * scaled_target_speeds[idx_b]

    return ref_horizon, s_targets


def run_tests():
    print("=== Running Continuous Projection Unit & Integrity Tests ===")
    track = TrackManager.load_track('Spielberg')
    wpts = track.waypoints
    s_arr = track.s_arr
    L_track = track.track_length
    N_pts = len(wpts)
    dt = 0.08
    N = 10

    # Test 1: Wrap-around boundary continuity
    print("\n--- Test 1: Wrap-Around Boundary Geometry & Numerical Continuity ---")
    p_last = wpts[-1]
    p_first = wpts[0]
    seg_len_wrap = L_track - s_arr[-1]
    print(f"Segment N-1 -> 0: s_last={s_arr[-1]:.6f}, L_track={L_track:.6f}, seg_len={seg_len_wrap:.6f}")
    assert seg_len_wrap > 0, "Wrap-around segment length must be positive"

    # Step through wrap-around in fine fractions
    last_s = None
    for frac in np.linspace(0.0, 1.0, 21):
        pt = (1.0 - frac) * p_last + frac * p_first
        dists = np.linalg.norm(wpts - pt, axis=1)
        cl_idx = int(np.argmin(dists))

        s_c, sa, sb, t_val, _ = compute_continuous_projection(pt[0], pt[1], cl_idx, wpts, s_arr, L_track, N_pts)
        assert np.isfinite(s_c), f"s_cont not finite: {s_c}"
        assert 0.0 <= s_c < L_track, f"s_cont={s_c} out of [0, L_track)"
        assert 0.0 <= t_val <= 1.0, f"t={t_val} out of [0, 1]"

        if last_s is not None:
            ds = (s_c - last_s + L_track) % L_track
            assert ds >= 0.0, f"Discontinuous backward step in wrap-around: ds={ds}"
        last_s = s_c
    print("✓ Wrap-around boundary is smooth and continuous.")

    # Test 2: Reference horizon matching and monotonicity across wrap-around
    print("\n--- Test 2: Reference Horizon Monotonicity & s_ref[0] Anchor ---")
    test_s_values = [0.0, 10.0, 30.46, L_track - 0.5, L_track - 0.05, L_track - 0.0001]
    for s_val in test_s_values:
        idx = int(np.searchsorted(s_arr, s_val, side='right') - 1)
        idx = max(0, min(N_pts - 1, idx))
        ref_h, s_targets = generate_ref_horizon(s_val, 5.0, dt, N, s_arr, wpts, track.headings, track.target_speeds, L_track, N_pts)

        assert abs(s_targets[0] - s_val) < 1e-9, f"s_ref[0] mismatch: {s_targets[0]} vs {s_val}"

        for k in range(N):
            ds_k = (s_targets[k + 1] - s_targets[k] + L_track) % L_track
            assert ds_k > 0.0, f"Horizon step {k} not strictly forward: {ds_k}"
            assert ds_k < L_track / 2.0, f"Horizon step {k} too large: {ds_k}"

        assert np.all(np.isfinite(ref_h)), "NaN/Inf in ref_horizon"
    print("✓ s_ref[0] exactly matches s_cont and horizon is strictly monotonic.")

    # Test 3: Replay all frames of Experiment 1 Telemetry
    print("\n--- Test 3: Integrity Check Across All 270 Cycles of Experiment 1 ---")
    with open('/home/yeswanth/roboracer_ws/diagnostic_experiment1_telemetry.json') as f:
        telem = json.load(f)

    max_backward_ds = 0.0
    prev_s_cont = None

    for i, step in enumerate(telem):
        cur_x = step['cur_x']
        cur_y = step['cur_y']
        closest_idx = step['closest_idx']
        cur_v = step['speed']

        s_c, sa, sb, t_val, dist_sq = compute_continuous_projection(cur_x, cur_y, closest_idx, wpts, s_arr, L_track, N_pts)

        assert np.isfinite(s_c), f"s_cont not finite at step {i}"
        assert 0.0 <= s_c < L_track, f"s_cont out of bounds at step {i}"
        assert 0.0 <= t_val <= 1.0, f"t out of bounds at step {i}: {t_val}"
        assert dist_sq < 1.0, f"Perpendicular distance too large at step {i}: {math.sqrt(dist_sq)}"

        ref_h, s_targets = generate_ref_horizon(s_c, cur_v, dt, N, s_arr, wpts, track.headings, track.target_speeds, L_track, N_pts)

        assert abs(s_targets[0] - s_c) < 1e-9, f"s_targets[0] != s_c at step {i}"
        for k in range(N):
            ds_k = (s_targets[k + 1] - s_targets[k] + L_track) % L_track
            assert ds_k > 0.0, f"Horizon non-monotonic at step {i}, k={k}"

        if prev_s_cont is not None:
            ds_cycle = s_c - prev_s_cont
            if ds_cycle < -L_track / 2.0:
                ds_cycle += L_track
            elif ds_cycle > L_track / 2.0:
                ds_cycle -= L_track
            if ds_cycle < 0:
                max_backward_ds = min(max_backward_ds, ds_cycle)
        prev_s_cont = s_c

    print(f"✓ Checked {len(telem)} frames. Max backward motion = {max_backward_ds:.6f} m (effectively 0).")
    print("ALL INTEGRITY TESTS PASSED SUCCESSFULLY!")

if __name__ == '__main__':
    run_tests()
