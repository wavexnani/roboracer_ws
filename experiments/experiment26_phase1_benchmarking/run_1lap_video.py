#!/usr/bin/env python3
"""
F1TENTH 1-Lap Simulation & Video Recording Engine

Supports:
1. Live GUI rendering with ego-following dynamic camera.
2. Direct-to-MP4 video recording with broadcast-style HUD telemetry banner.
3. Automated batch recording for all 17 benchmark scenarios across 5 controllers and 3 tracks.
"""

import os
import sys
import math
import time
import argparse
import subprocess
from typing import Optional, Dict, Any, List
import numpy as np
import cv2

# Set working directory and path
EXP_DIR = os.path.dirname(os.path.abspath(__file__))
if EXP_DIR not in sys.path:
    sys.path.insert(0, EXP_DIR)
try:
    sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass

import run_phase1_benchmarking as bench
import gym
import pyglet
import f110_gym
from f110_gym.envs.f110_env import F110Env
from mpc_controller.track_manager import TrackManager
from controllers.pure_pursuit_adapter import PurePursuitAdapter

DEFAULT_VIDEO_DIR = os.path.join(EXP_DIR, 'videos')

BENCHMARK_SCENARIOS = [
    # Primary controllers
    {"controller": "MPC_Controller", "track": "Levine", "tuned_pp": False, "filename": "mpc_levine_1lap.mp4", "expected": "Complete (1 Lap)"},
    {"controller": "MPC_Controller", "track": "Spielberg", "tuned_pp": False, "filename": "mpc_spielberg_1lap.mp4", "expected": "Complete (1 Lap)"},
    {"controller": "MPC_Controller", "track": "Monza", "tuned_pp": False, "filename": "mpc_monza_1lap.mp4", "expected": "Complete (1 Lap)"},
    {"controller": "Stanley_Controller", "track": "Levine", "tuned_pp": False, "filename": "stanley_levine_1lap.mp4", "expected": "Complete (1 Lap)"},
    {"controller": "Stanley_Controller", "track": "Spielberg", "tuned_pp": False, "filename": "stanley_spielberg_1lap.mp4", "expected": "Complete (1 Lap)"},
    {"controller": "Stanley_Controller", "track": "Monza", "tuned_pp": False, "filename": "stanley_monza_1lap.mp4", "expected": "Complete (1 Lap)"},

    # Reactive & Secondary controllers
    {"controller": "Follow_The_Gap", "track": "Levine", "tuned_pp": False, "filename": "ftg_levine_1lap.mp4", "expected": "Complete (1 Lap)"},
    {"controller": "Follow_The_Gap", "track": "Spielberg", "tuned_pp": False, "filename": "ftg_spielberg_1lap.mp4", "expected": "Complete (1 Lap)"},
    {"controller": "Follow_The_Gap", "track": "Monza", "tuned_pp": False, "filename": "ftg_monza_1lap.mp4", "expected": "Complete (1 Lap)"},
    {"controller": "Pure_Pursuit", "track": "Levine", "tuned_pp": False, "filename": "pure_pursuit_baseline_levine_1lap.mp4", "expected": "Complete (1 Lap)"},
    {"controller": "Pure_Pursuit", "track": "Spielberg", "tuned_pp": False, "filename": "pure_pursuit_baseline_spielberg_crash.mp4", "expected": "Crash"},
    {"controller": "Pure_Pursuit", "track": "Monza", "tuned_pp": False, "filename": "pure_pursuit_baseline_monza_crash.mp4", "expected": "Crash"},
    {"controller": "Pure_Pursuit", "track": "Spielberg", "tuned_pp": True, "filename": "pure_pursuit_tuned_spielberg_1lap.mp4", "expected": "Complete (1 Lap)"},
    {"controller": "Pure_Pursuit", "track": "Monza", "tuned_pp": True, "filename": "pure_pursuit_tuned_monza_1lap.mp4", "expected": "Complete (1 Lap)"},
    {"controller": "Wall_Following", "track": "Levine", "tuned_pp": False, "filename": "wall_following_levine_crash.mp4", "expected": "Crash (90 deg Turn)"},
    {"controller": "Wall_Following", "track": "Spielberg", "tuned_pp": False, "filename": "wall_following_spielberg_1lap.mp4", "expected": "Complete (1 Lap)"},
    {"controller": "Wall_Following", "track": "Monza", "tuned_pp": False, "filename": "wall_following_monza_1lap.mp4", "expected": "Complete (1 Lap)"},
]


def render_and_record_run(
    controller_name: str,
    track_name: str,
    target_laps: int = 1,
    tuned_pp: bool = False,
    render_mode: str = 'human_fast',
    record_video: bool = True,
    output_path: Optional[str] = None,
    max_sim_time: float = 60.0
) -> Dict[str, Any]:
    """Executes a single simulation run with camera centering and video recording."""
    os.makedirs(DEFAULT_VIDEO_DIR, exist_ok=True)
    if output_path is None and record_video:
        suffix = "_tuned" if tuned_pp else ""
        output_path = os.path.join(DEFAULT_VIDEO_DIR, f"{controller_name.lower()}{suffix}_{track_name.lower()}_{target_laps}lap.mp4")

    track_info = TrackManager.load_track(track_name, 'raceline')

    # Controller instantiation
    if controller_name == 'Pure_Pursuit' and tuned_pp:
        v_max = 4.20 if track_name == 'Levine' else (7.50 if track_name == 'Monza' else 7.00)
        scale = 0.85 if track_name == 'Levine' else (0.75 if track_name == 'Monza' else 0.70)
        ctrl = PurePursuitAdapter(
            waypoints=track_info.waypoints,
            headings=track_info.headings,
            target_speeds=track_info.target_speeds,
            lookahead_distance=0.80,
            lookahead_gain=0.12,
            min_lookahead=0.60,
            max_lookahead=2.20,
            speed_scale=scale,
            max_speed=v_max
        )
    else:
        ctrl = bench.instantiate_controller(controller_name, track_info, track_name)

    ctrl.reset()

    # Reset renderer singleton to ensure clean state
    if F110Env.renderer is not None:
        try:
            F110Env.renderer.close()
        except Exception:
            pass
        F110Env.renderer = None

    env = gym.make('f110_gym:f110-v0', map=track_info.map_path_no_ext, map_ext='.png', num_agents=1)
    reset_ret = env.reset(np.array([[track_info.start_pose[0], track_info.start_pose[1], track_info.start_pose[2]]]))
    obs = reset_ret[0] if isinstance(reset_ret, tuple) else reset_ret

    sim_dt = 0.010
    control_rate_hz = 25.0
    control_interval = int(round((1.0 / control_rate_hz) / sim_dt))  # 4 steps
    control_dt = 1.0 / control_rate_hz

    step_count = 0
    last_lap_count = int(obs['lap_counts'][0])
    completed_laps = 0
    steer_cmd, speed_cmd = 0.0, 0.0
    crashed = False
    crash_reason = "None"

    ffmpeg_proc = None
    frame_w, frame_h = 1000, 800

    if record_video:
        cmd = [
            'ffmpeg', '-y', '-f', 'rawvideo', '-vcodec', 'rawvideo',
            '-s', f'{frame_w}x{frame_h}', '-pix_fmt', 'bgr24',
            '-r', str(int(control_rate_hz)),
            '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
            '-crf', '22', '-preset', 'veryfast', output_path
        ]
        ffmpeg_proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)

    print(f"\n[RUN] {controller_name} on {track_name} (Target: {target_laps} lap(s), Tuned: {tuned_pp})")
    if record_video:
        print(f"      Recording to: {output_path}")

    # Main simulation loop
    while (step_count * sim_dt) < max_sim_time:
        cur_x = float(obs['poses_x'][0])
        cur_y = float(obs['poses_y'][0])
        cur_yaw = float(obs['poses_theta'][0])
        cur_v = float(math.hypot(obs['linear_vels_x'][0], obs['linear_vels_y'][0]))
        current_lap_count = int(obs['lap_counts'][0])
        scans = obs['scans'][0] if len(obs['scans'][0]) > 0 else []

        # Collision detection
        is_collision = bool(obs['collisions'][0])
        min_scan = float(np.min(scans)) if len(scans) > 0 else 10.0
        if is_collision or min_scan < 0.12:
            crashed = True
            crash_reason = f"Collision at t={step_count*sim_dt:.2f}s (min_scan={min_scan:.2f}m)"
            print(f"      --> {crash_reason}")
            break

        # Stall detection
        if step_count > 600 and cur_v < 0.10:
            crashed = True
            crash_reason = "Vehicle stalled at start line"
            print(f"      --> {crash_reason}")
            break

        # Lap completion detection
        if current_lap_count > last_lap_count:
            completed_laps += 1
            last_lap_count = current_lap_count
            print(f"      --> Completed lap {completed_laps}/{target_laps} at t={step_count*sim_dt:.2f}s")
            if completed_laps >= target_laps:
                break

        # Compute control
        if step_count % control_interval == 0:
            steer_cmd, speed_cmd, _, _ = ctrl.compute_control(cur_x, cur_y, cur_yaw, cur_v, control_dt, scans=scans)

        # Simulation step
        step_ret = env.step(np.array([[steer_cmd, speed_cmd]]))
        obs = step_ret[0] if isinstance(step_ret, tuple) else step_ret

        # Render & record at control rate (25 FPS)
        if step_count % control_interval == 0:
            env.render(mode=render_mode)
            renderer = F110Env.renderer

            if renderer is not None:
                # Camera follows ego vehicle
                cx, cy = 50.0 * cur_x, 50.0 * cur_y
                renderer.left = cx - renderer.zoomed_width / 2
                renderer.right = cx + renderer.zoomed_width / 2
                renderer.bottom = cy - renderer.zoomed_height / 2
                renderer.top = cy + renderer.zoomed_height / 2

            if record_video and ffmpeg_proc is not None:
                color_buf = pyglet.image.get_buffer_manager().get_color_buffer()
                img_data = color_buf.get_image_data()
                raw = img_data.get_data('RGB', img_data.width * 3)
                arr = np.frombuffer(raw, dtype=np.uint8).reshape(img_data.height, img_data.width, 3)
                frame = cv2.cvtColor(np.flipud(arr), cv2.COLOR_RGB2BGR)

                # Draw broadcast HUD telemetry bar
                overlay = frame.copy()
                cv2.rectangle(overlay, (10, 10), (990, 52), (18, 22, 34), -1)
                cv2.addWeighted(overlay, 0.80, frame, 0.20, 0, frame)
                cv2.rectangle(frame, (10, 10), (990, 52), (65, 85, 130), 1)

                display_ctrl = f"{controller_name} (Tuned)" if tuned_pp else controller_name
                txt = f"{display_ctrl} | {track_name} | Lap: {step_count*sim_dt:5.2f}s | Speed: {cur_v:4.2f} m/s | Steer: {np.degrees(steer_cmd):+5.1f} deg"
                cv2.putText(frame, txt, (22, 37), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (245, 245, 245), 2, cv2.LINE_AA)

                ffmpeg_proc.stdin.write(frame.tobytes())

        step_count += 1

    # In case of crash, capture 15 frames of the crash site
    if crashed and record_video and ffmpeg_proc is not None:
        for _ in range(15):
            overlay = frame.copy()
            cv2.rectangle(overlay, (250, 360), (750, 440), (20, 20, 120), -1)
            cv2.addWeighted(overlay, 0.85, frame, 0.15, 0, frame)
            cv2.rectangle(frame, (250, 360), (750, 440), (50, 50, 240), 2)
            cv2.putText(frame, "COLLISION DETECTED", (290, 412), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
            ffmpeg_proc.stdin.write(frame.tobytes())

    if ffmpeg_proc is not None:
        ffmpeg_proc.stdin.close()
        ffmpeg_proc.wait()

    if F110Env.renderer is not None:
        try:
            F110Env.renderer.close()
        except Exception:
            pass
        F110Env.renderer = None

    result = {
        "controller": controller_name,
        "track": track_name,
        "tuned": tuned_pp,
        "completed": completed_laps >= target_laps,
        "crashed": crashed,
        "crash_reason": crash_reason,
        "elapsed_sim_time": round(step_count * sim_dt, 2),
        "video_path": output_path if record_video else None
    }
    return result


def main():
    parser = argparse.ArgumentParser(description="F1TENTH 1-Lap Simulation & Video Recording Engine")
    parser.add_argument('--controller', default='MPC_Controller', choices=bench.CONTROLLERS + ['all'], help='Target controller')
    parser.add_argument('--track', default='Levine', choices=bench.TRACKS + ['all'], help='Target track')
    parser.add_argument('--laps', type=int, default=1, help='Lap count (default: 1)')
    parser.add_argument('--render-mode', default='human_fast', choices=['human', 'human_fast'], help='Rendering speed')
    parser.add_argument('--tuned-pp', action='store_true', help='Use Experiment 1.2 speed-adaptive lookahead for Pure Pursuit')
    parser.add_argument('--record', action='store_true', default=True, help='Record video to MP4')
    parser.add_argument('--no-record', dest='record', action='store_false', help='Disable video recording')
    parser.add_argument('--output', default=None, help='Output MP4 video file path')
    parser.add_argument('--batch', action='store_true', help='Execute all 17 documented benchmark scenarios')

    args = parser.parse_args()

    if args.batch:
        print("\n" + "="*70)
        print("  STARTING AUTOMATED 17-SCENARIO BENCHMARK VIDEO RECORDING")
        print("="*70)
        os.makedirs(DEFAULT_VIDEO_DIR, exist_ok=True)
        results = []
        for idx, sc in enumerate(BENCHMARK_SCENARIOS, 1):
            print(f"\n[{idx}/17] Running {sc['controller']} on {sc['track']} (Expected: {sc['expected']})...")
            out_file = os.path.join(DEFAULT_VIDEO_DIR, sc['filename'])
            res = render_and_record_run(
                controller_name=sc['controller'],
                track_name=sc['track'],
                target_laps=1,
                tuned_pp=sc['tuned_pp'],
                render_mode=args.render_mode,
                record_video=True,
                output_path=out_file
            )
            res['expected'] = sc['expected']
            results.append(res)

        print("\n" + "="*70)
        print("  BATCH RECORDING COMPLETE SUMMARY")
        print("="*70)
        print(f"{'#':<3} {'Controller':<20} {'Track':<11} {'Status':<12} {'Sim Time':<10} {'Video File'}")
        print("-" * 75)
        for i, r in enumerate(results, 1):
            st = "Lap Done" if r['completed'] else ("Crash" if r['crashed'] else "Timeout")
            fname = os.path.basename(r['video_path']) if r['video_path'] else "None"
            print(f"{i:<3} {r['controller']:<20} {r['track']:<11} {st:<12} {r['elapsed_sim_time']:<10.2f} {fname}")
        print("="*70 + "\n")
        return

    # Single run mode
    render_and_record_run(
        controller_name=args.controller,
        track_name=args.track,
        target_laps=args.laps,
        tuned_pp=args.tuned_pp,
        render_mode=args.render_mode,
        record_video=args.record,
        output_path=args.output
    )


if __name__ == '__main__':
    main()
