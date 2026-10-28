#!/usr/bin/env python3
"""
F1TENTH RViz Simulation & Video Recording Engine

Records autonomous laps using the actual ROS 2 RViz visualizer:
1. Launches f1tenth_gym_ros with RViz2 (3D car model, LiDAR rays, map, autofocus camera).
2. Runs the selected ROS 2 controller node communicating over /ego_racecar/odom, /scan, and /drive.
3. Automatically finds the RViz window on the display and captures it to H.264 MP4 using FFmpeg x11grab.
4. Detects 1-lap completion and collisions in real-time, cleanly stopping and saving the video.
"""

import os
import sys
import re
import math
import time
import signal
import argparse
import subprocess
from typing import Optional, Dict, Any, Tuple

# ROS 2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

# Workspace & Benchmark imports
EXP_DIR = os.path.dirname(os.path.abspath(__file__))
if EXP_DIR not in sys.path:
    sys.path.insert(0, EXP_DIR)

import run_phase1_benchmarking as bench
from mpc_controller.track_manager import TrackManager
from controllers.pure_pursuit_adapter import PurePursuitAdapter

try:
    sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass

VIDEOS_DIR = os.path.join(EXP_DIR, 'videos_rviz')

BENCHMARK_SCENARIOS = [
    # Primary controllers
    {"controller": "MPC_Controller", "track": "Levine", "tuned_pp": False, "filename": "rviz_mpc_levine.mp4", "expected": "Complete (1 Lap)"},
    {"controller": "MPC_Controller", "track": "Spielberg", "tuned_pp": False, "filename": "rviz_mpc_spielberg.mp4", "expected": "Complete (1 Lap)"},
    {"controller": "MPC_Controller", "track": "Monza", "tuned_pp": False, "filename": "rviz_mpc_monza.mp4", "expected": "Complete (1 Lap)"},
    {"controller": "Stanley_Controller", "track": "Levine", "tuned_pp": False, "filename": "rviz_stanley_levine.mp4", "expected": "Complete (1 Lap)"},
    {"controller": "Stanley_Controller", "track": "Spielberg", "tuned_pp": False, "filename": "rviz_stanley_spielberg.mp4", "expected": "Complete (1 Lap)"},
    {"controller": "Stanley_Controller", "track": "Monza", "tuned_pp": False, "filename": "rviz_stanley_monza.mp4", "expected": "Complete (1 Lap)"},

    # Reactive & Secondary controllers
    {"controller": "Follow_The_Gap", "track": "Levine", "tuned_pp": False, "filename": "rviz_ftg_levine.mp4", "expected": "Complete (1 Lap)"},
    {"controller": "Follow_The_Gap", "track": "Spielberg", "tuned_pp": False, "filename": "rviz_ftg_spielberg.mp4", "expected": "Complete (1 Lap)"},
    {"controller": "Follow_The_Gap", "track": "Monza", "tuned_pp": False, "filename": "rviz_ftg_monza.mp4", "expected": "Complete (1 Lap)"},
    {"controller": "Pure_Pursuit", "track": "Levine", "tuned_pp": False, "filename": "rviz_pure_pursuit_baseline_levine.mp4", "expected": "Complete (1 Lap)"},
    {"controller": "Pure_Pursuit", "track": "Spielberg", "tuned_pp": False, "filename": "rviz_pure_pursuit_baseline_spielberg_crash.mp4", "expected": "Crash"},
    {"controller": "Pure_Pursuit", "track": "Monza", "tuned_pp": False, "filename": "rviz_pure_pursuit_baseline_monza_crash.mp4", "expected": "Crash"},
    {"controller": "Pure_Pursuit", "track": "Spielberg", "tuned_pp": True, "filename": "rviz_pure_pursuit_tuned_spielberg.mp4", "expected": "Complete (1 Lap)"},
    {"controller": "Pure_Pursuit", "track": "Monza", "tuned_pp": True, "filename": "rviz_pure_pursuit_tuned_monza.mp4", "expected": "Complete (1 Lap)"},
    {"controller": "Wall_Following", "track": "Levine", "tuned_pp": False, "filename": "rviz_wall_following_levine_crash.mp4", "expected": "Crash (90 deg Turn)"},
    {"controller": "Wall_Following", "track": "Spielberg", "tuned_pp": False, "filename": "rviz_wall_following_spielberg.mp4", "expected": "Complete (1 Lap)"},
    {"controller": "Wall_Following", "track": "Monza", "tuned_pp": False, "filename": "rviz_wall_following_monza.mp4", "expected": "Complete (1 Lap)"},
]


def find_rviz_geometry(max_wait_s: float = 12.0) -> Optional[Tuple[str, int, int, int, int]]:
    """Polls for the active RViz2 window and returns (wid, x, y, width, height)."""
    start_t = time.time()
    while time.time() - start_t < max_wait_s:
        try:
            out = subprocess.check_output(['wmctrl', '-l'], stderr=subprocess.DEVNULL).decode()
            for line in out.strip().split('\n'):
                if 'rviz' in line.lower():
                    wid = line.split()[0]
                    info = subprocess.check_output(['xwininfo', '-id', wid], stderr=subprocess.DEVNULL).decode()
                    m_x = re.search(r'Absolute upper-left X:\s+(-?\d+)', info)
                    m_y = re.search(r'Absolute upper-left Y:\s+(-?\d+)', info)
                    m_w = re.search(r'Width:\s+(\d+)', info)
                    m_h = re.search(r'Height:\s+(\d+)', info)
                    if m_x and m_y and m_w and m_h:
                        x = max(0, int(m_x.group(1)))
                        y = max(0, int(m_y.group(1)))
                        w = int(m_w.group(1))
                        h = int(m_h.group(1))
                        # Even width & height required for H.264
                        w = w if w % 2 == 0 else w - 1
                        h = h if h % 2 == 0 else h - 1
                        return wid, x, y, w, h
        except Exception:
            pass
        time.sleep(0.3)
    return None


class RVizLapRunnerNode(Node):
    """ROS 2 Node controlling the car and tracking lap status in RViz."""

    def __init__(self, controller_name: str, track_name: str, tuned_pp: bool = False, max_time_s: float = 60.0):
        super().__init__('rviz_lap_runner_node')

        self.controller_name = controller_name
        self.track_name = track_name
        self.max_time_s = max_time_s
        self.track_info = TrackManager.load_track(track_name, 'raceline')
        self.track_length = self.track_info.track_length
        self.start_pose = self.track_info.start_pose

        # Instantiate target controller adapter
        if controller_name == 'Pure_Pursuit' and tuned_pp:
            v_max = 4.20 if track_name == 'Levine' else (7.50 if track_name == 'Monza' else 7.00)
            scale = 0.85 if track_name == 'Levine' else (0.75 if track_name == 'Monza' else 0.70)
            self.ctrl = PurePursuitAdapter(
                waypoints=self.track_info.waypoints,
                headings=self.track_info.headings,
                target_speeds=self.track_info.target_speeds,
                lookahead_distance=0.80,
                lookahead_gain=0.12,
                min_lookahead=0.60,
                max_lookahead=2.20,
                speed_scale=scale,
                max_speed=v_max
            )
        else:
            self.ctrl = bench.instantiate_controller(controller_name, self.track_info, track_name)

        self.ctrl.reset()

        # Telemetry & State
        self.cur_x = self.start_pose[0]
        self.cur_y = self.start_pose[1]
        self.cur_yaw = self.start_pose[2]
        self.cur_v = 0.0
        self.scans = []
        self.total_dist = 0.0
        self.prev_xy = (self.cur_x, self.cur_y)
        self.start_time = None
        self.elapsed_time = 0.0

        self.lap_completed = False
        self.crashed = False
        self.crash_reason = "None"
        self.finished = False

        # Subscriptions & Publisher
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # Control loop at 25 Hz (0.04s)
        self.timer = self.create_timer(0.040, self.control_loop)

    def scan_cb(self, msg: LaserScan):
        self.scans = list(msg.ranges)

    def odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.cur_x = pos.x
        self.cur_y = pos.y

        # Quaternion to yaw
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        self.cur_yaw = math.atan2(siny_cosp, cosy_cosp)

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.cur_v = math.hypot(vx, vy)

        # Distance accumulator
        step_d = math.hypot(self.cur_x - self.prev_xy[0], self.cur_y - self.prev_xy[1])
        if step_d > 0.001:
            self.total_dist += step_d
            self.prev_xy = (self.cur_x, self.cur_y)

    def control_loop(self):
        if self.finished:
            return

        now = time.time()
        if self.start_time is None:
            self.start_time = now

        self.elapsed_time = now - self.start_time

        # Check collision via LiDAR
        min_scan = min(self.scans) if self.scans else 10.0
        if min_scan < 0.12 and self.total_dist > 0.5:
            self.crashed = True
            self.crash_reason = f"Collision (min_scan={min_scan:.2f}m at dist={self.total_dist:.1f}m)"
            self.finished = True
            self.stop_vehicle()
            return

        # Check vehicle stall
        if self.elapsed_time > 6.0 and self.total_dist < 0.5:
            self.crashed = True
            self.crash_reason = "Vehicle stalled at starting line"
            self.finished = True
            self.stop_vehicle()
            return

        # Check lap completion
        d_to_start = math.hypot(self.cur_x - self.start_pose[0], self.cur_y - self.start_pose[1])
        if self.total_dist > (self.track_length * 0.82) and d_to_start < 3.0:
            self.lap_completed = True
            self.finished = True
            print(f"      [LAP DONE] Completed 1 lap in {self.elapsed_time:.2f}s (Total dist: {self.total_dist:.1f}m)")
            self.stop_vehicle()
            return

        # Timeout safety
        if self.elapsed_time > self.max_time_s:
            self.crashed = False
            self.crash_reason = "Max timeout reached"
            self.finished = True
            self.stop_vehicle()
            return

        # Compute control
        steer_cmd, speed_cmd, _, _ = self.ctrl.compute_control(
            self.cur_x, self.cur_y, self.cur_yaw, self.cur_v, 0.040, scans=self.scans
        )

        # Publish drive command
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'ego_racecar/base_link'
        drive_msg.drive.steering_angle = float(steer_cmd)
        drive_msg.drive.speed = float(speed_cmd)
        self.drive_pub.publish(drive_msg)

    def stop_vehicle(self):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'ego_racecar/base_link'
        drive_msg.drive.steering_angle = 0.0
        drive_msg.drive.speed = 0.0
        self.drive_pub.publish(drive_msg)


def record_scenario_in_rviz(
    scenario: Dict[str, Any],
    output_dir: str,
    max_sim_time: float = 60.0
) -> Dict[str, Any]:
    """Runs a complete RViz simulation and records the RViz window to MP4."""
    os.makedirs(output_dir, exist_ok=True)
    ctrl_name = scenario['controller']
    track = scenario['track']
    tuned = scenario['tuned_pp']
    out_file = os.path.join(output_dir, scenario['filename'])

    print(f"\n{'='*70}")
    print(f"  STARTING RVIZ RUN: {ctrl_name} on {track}")
    print(f"  Output: {out_file} (Expected: {scenario['expected']})")
    print(f"{'='*70}")

    # 1. Clean up any previous ROS 2 nodes
    subprocess.run(['pkill', '-9', '-f', 'rviz2'], stderr=subprocess.DEVNULL)
    subprocess.run(['pkill', '-9', '-f', 'gym_bridge'], stderr=subprocess.DEVNULL)
    subprocess.run(['pkill', '-9', '-f', 'robot_state_publisher'], stderr=subprocess.DEVNULL)
    subprocess.run(['pkill', '-9', '-f', 'map_server'], stderr=subprocess.DEVNULL)
    subprocess.run(['pkill', '-9', '-f', 'lifecycle_manager'], stderr=subprocess.DEVNULL)
    time.sleep(1.0)

    # 2. Launch the F1TENTH gym ROS bridge + RViz2
    launch_cmd = [
        'bash', '-c',
        f'source /opt/ros/jazzy/setup.bash && source /home/yeswanth/roboracer_ws/install/setup.bash && '
        f'ros2 launch f1tenth_gym_ros gym_bridge_launch.py map:={track} autofocus:=true'
    ]
    sim_proc = subprocess.Popen(launch_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)

    # 3. Locate the RViz2 window geometry
    print("  --> Waiting for RViz2 window to appear on screen...")
    geo = find_rviz_geometry(max_wait_s=15.0)
    if not geo:
        print("  [ERROR] Could not detect RViz2 window on DISPLAY :1")
        os.killpg(os.getpgid(sim_proc.pid), signal.SIGTERM)
        return {"scenario": scenario, "success": False, "error": "RViz window not found"}

    wid, win_x, win_y, win_w, win_h = geo
    print(f"  --> RViz2 window detected: {win_w}x{win_h} at +{win_x}+{win_y} (ID: {wid})")

    # 4. Start FFmpeg screen recorder on the RViz window coordinates
    display_id = os.environ.get('DISPLAY', ':1')
    ffmpeg_cmd = [
        'ffmpeg', '-y',
        '-f', 'x11grab',
        '-framerate', '30',
        '-video_size', f'{win_w}x{win_h}',
        '-i', f'{display_id}.0+{win_x},{win_y}',
        '-c:v', 'libx264',
        '-pix_fmt', 'yuv420p',
        '-preset', 'veryfast',
        '-crf', '22',
        out_file
    ]
    ffmpeg_proc = subprocess.Popen(ffmpeg_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print(f"  --> FFmpeg screen recorder attached to RViz window.")

    # 5. Spin ROS 2 controller node until finished
    rclpy.init()
    node = RVizLapRunnerNode(ctrl_name, track, tuned_pp=tuned, max_time_s=max_sim_time)
    print(f"  --> Controller node running. Tracking lap progress in RViz...")

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.04)
    except KeyboardInterrupt:
        print("  --> User interrupted.")

    # Allow 2 extra seconds of recording for finish/crash capture
    time.sleep(2.0)

    # 6. Stop recorder & clean up
    print("  --> Stopping recorder and simulation...")
    ffmpeg_proc.send_signal(signal.SIGINT)
    try:
        ffmpeg_proc.wait(timeout=5.0)
    except Exception:
        ffmpeg_proc.kill()

    node.destroy_node()
    rclpy.shutdown()

    try:
        os.killpg(os.getpgid(sim_proc.pid), signal.SIGTERM)
        sim_proc.wait(timeout=3.0)
    except Exception:
        pass

    # Clean any lingering nodes
    subprocess.run(['pkill', '-9', '-f', 'rviz2'], stderr=subprocess.DEVNULL)
    subprocess.run(['pkill', '-9', '-f', 'gym_bridge'], stderr=subprocess.DEVNULL)

    status = "Lap Completed" if node.lap_completed else (f"Crashed: {node.crash_reason}" if node.crashed else "Finished")
    print(f"  [RESULT] {ctrl_name} on {track}: {status} (Time: {node.elapsed_time:.2f}s)")

    return {
        "scenario": scenario,
        "completed": node.lap_completed,
        "crashed": node.crashed,
        "crash_reason": node.crash_reason,
        "elapsed_time": round(node.elapsed_time, 2),
        "video_path": out_file
    }


def main():
    parser = argparse.ArgumentParser(description="F1TENTH RViz Simulation & Video Recording Engine")
    parser.add_argument('--scenario', type=int, default=None, help='Index (1-17) of specific scenario to record')
    parser.add_argument('--controller', default=None, help='Filter by controller name')
    parser.add_argument('--track', default=None, help='Filter by track name')
    parser.add_argument('--batch', action='store_true', default=True, help='Record all 17 scenarios sequentially')

    args = parser.parse_args()

    scenarios = BENCHMARK_SCENARIOS
    if args.scenario is not None:
        scenarios = [BENCHMARK_SCENARIOS[args.scenario - 1]]
    elif args.controller or args.track:
        scenarios = [
            s for s in BENCHMARK_SCENARIOS
            if (not args.controller or args.controller.lower() in s['controller'].lower()) and
               (not args.track or args.track.lower() == s['track'].lower())
        ]

    print("\n" + "="*70)
    print(f"  F1TENTH RVIZ BATCH VIDEO RECORDER ({len(scenarios)} SCENARIOS)")
    print(f"  Target Videos Directory: {VIDEOS_DIR}")
    print("="*70)

    results = []
    for idx, sc in enumerate(scenarios, 1):
        print(f"\n>>> Running Scenario [{idx}/{len(scenarios)}]: {sc['controller']} on {sc['track']}...")
        res = record_scenario_in_rviz(sc, VIDEOS_DIR)
        results.append(res)
        time.sleep(2.0)

    print("\n" + "="*70)
    print("  ALL RVIZ RECORDINGS COMPLETE SUMMARY")
    print("="*70)
    print(f"{'#':<3} {'Controller':<20} {'Track':<11} {'Status':<14} {'Time':<8} {'Video File'}")
    print("-" * 75)
    for i, r in enumerate(results, 1):
        sc = r['scenario']
        st = "Lap Done" if r.get('completed') else ("Crash" if r.get('crashed') else "Incomplete")
        fname = os.path.basename(r.get('video_path', ''))
        t = f"{r.get('elapsed_time', 0.0):.1f}s"
        print(f"{i:<3} {sc['controller']:<20} {sc['track']:<11} {st:<14} {t:<8} {fname}")
    print("="*70 + "\n")


if __name__ == '__main__':
    main()
