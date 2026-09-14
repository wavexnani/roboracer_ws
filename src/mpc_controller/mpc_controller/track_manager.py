#!/usr/bin/env python3
"""
Scalable Track & Map Manager for F1TENTH / RoboRacer.

Provides unified track discovery, dual-format waypoint ingestion (racelines & centerlines),
dynamic map loading across all 24 f1tenth_racetracks maps, and simulator synchronization.
Designed to be reusable across all trajectory-tracking controllers (Stanley, Pure Pursuit, MPC).
"""

import os
import glob
import math
import numpy as np
import yaml
from dataclasses import dataclass
from typing import Optional, Tuple, List

try:
    from scipy.ndimage import gaussian_filter1d
    HAS_GAUSSIAN_FILTER = True
except ImportError:
    HAS_GAUSSIAN_FILTER = False


@dataclass
class TrackInfo:
    """Encapsulates all metadata and trajectory profiles for a racetrack."""
    track_name: str
    track_dir: str
    csv_path: str
    waypoint_type: str  # 'raceline' or 'centerline'
    waypoints: np.ndarray      # (N, 2) [x, y] in meters
    headings: np.ndarray       # (N,) psi in radians [-pi, pi]
    s_arr: np.ndarray          # (N,) cumulative arc length in meters
    kappa: np.ndarray          # (N,) path curvature [rad/m]
    target_speeds: np.ndarray  # (N,) target longitudinal speeds [m/s]
    track_length: float        # Total closed-loop track length [m]
    start_pose: Tuple[float, float, float]  # (x0, y0, theta0)
    map_yaml_path: Optional[str] = None
    map_png_path: Optional[str] = None
    map_path_no_ext: Optional[str] = None


class TrackManager:
    """Manages discovery, loading, and simulator synchronization for F1TENTH racetracks."""

    @staticmethod
    def get_workspace_root() -> str:
        """Finds the workspace root directory (/home/.../roboracer_ws)."""
        current = os.path.abspath(__file__)
        # Walk up until finding 'src' or workspace markers
        while current and current != os.path.dirname(current):
            if os.path.isdir(os.path.join(current, 'src', 'f1tenth_racetracks')):
                return current
            if os.path.basename(current) in ('roboracer_ws', 'sim_ws'):
                return current
            current = os.path.dirname(current)
        # Fallbacks
        for cand in ['/home/yeswanth/roboracer_ws', '/sim_ws']:
            if os.path.isdir(cand):
                return cand
        return os.getcwd()

    @classmethod
    def get_racetracks_root(cls) -> str:
        """Returns the absolute path to f1tenth_racetracks directory."""
        ws_root = cls.get_workspace_root()
        cand = os.path.join(ws_root, 'src', 'f1tenth_racetracks')
        if os.path.isdir(cand):
            return cand
        # Search relative to current script
        script_dir = os.path.dirname(os.path.abspath(__file__))
        cand2 = os.path.abspath(os.path.join(script_dir, '../../f1tenth_racetracks'))
        if os.path.isdir(cand2):
            return cand2
        return '/home/yeswanth/roboracer_ws/src/f1tenth_racetracks'

    @classmethod
    def list_available_tracks(cls) -> List[str]:
        """Returns a sorted list of all available racetrack map names."""
        racetracks_root = cls.get_racetracks_root()
        if not os.path.isdir(racetracks_root):
            return []
        return sorted([
            d for d in os.listdir(racetracks_root)
            if os.path.isdir(os.path.join(racetracks_root, d)) and not d.startswith('.')
        ])

    @classmethod
    def resolve_track_name(cls, query: str) -> Optional[str]:
        """
        Fuzzy case-insensitive matching for racetrack names.
        Examples: 'austin' -> 'Austin', 'mexico' -> 'Mexico City', 'brands_hatch' -> 'BrandsHatch'.
        """
        if not query:
            return None
        # Reject CLI option flags (e.g. -r, -p, --ros-args)
        if query.startswith('-'):
            return None
        tracks = cls.list_available_tracks()
        q_norm = query.lower().replace(' ', '').replace('_', '').replace('-', '')
        if not q_norm:
            return None

        # Exact normalized match
        for t in tracks:
            if t.lower().replace(' ', '').replace('_', '').replace('-', '') == q_norm:
                return t

        # Substring normalized match (require at least 3 characters to prevent false positives from single letters)
        if len(q_norm) >= 3:
            for t in tracks:
                t_norm = t.lower().replace(' ', '').replace('_', '').replace('-', '')
                if q_norm in t_norm:
                    return t

        return None

    @classmethod
    def load_track(
        cls,
        track_name: str = 'Spielberg',
        waypoint_type: str = 'raceline',
        custom_csv_path: Optional[str] = None,
        default_speed: float = 7.5,
        max_straight_speed: float = 7.5,
        min_corner_speed: float = 1.8,
        lat_accel_max: float = 2.5,
        a_brake_max: float = 2.2,
        a_accel_max: float = 2.5
    ) -> TrackInfo:
        """
        Loads track waypoints, geometry, and map paths for any map in f1tenth_racetracks.
        Handles both 7-column racelines and 4-column centerlines seamlessly.
        """
        racetracks_root = cls.get_racetracks_root()
        ws_root = cls.get_workspace_root()

        canonical_name = cls.resolve_track_name(track_name)
        if canonical_name is None:
            canonical_name = track_name

        track_dir = os.path.join(racetracks_root, canonical_name)
        active_csv = None
        actual_type = waypoint_type.lower()

        # If custom CSV path given and valid, use it
        if custom_csv_path and os.path.isfile(custom_csv_path):
            active_csv = custom_csv_path
            actual_type = 'custom'
        elif os.path.isdir(track_dir):
            # Try requested type first
            if actual_type == 'raceline':
                cand_rl = glob.glob(os.path.join(track_dir, '*raceline.csv'))
                if cand_rl:
                    active_csv = cand_rl[0]
                else:
                    # Fallback to centerline if raceline does not exist (e.g. Montreal, Shanghai)
                    cand_cl = glob.glob(os.path.join(track_dir, '*centerline.csv'))
                    if cand_cl:
                        active_csv = cand_cl[0]
                        actual_type = 'centerline'
            else:
                cand_cl = glob.glob(os.path.join(track_dir, '*centerline.csv'))
                if cand_cl:
                    active_csv = cand_cl[0]
                    actual_type = 'centerline'
                else:
                    cand_rl = glob.glob(os.path.join(track_dir, '*raceline.csv'))
                    if cand_rl:
                        active_csv = cand_rl[0]
                        actual_type = 'raceline'

        # Fallback to local stanley_controller waypoints folder
        if active_csv is None or not os.path.isfile(active_csv):
            local_cands = [
                os.path.join(ws_root, f'src/stanley_controller/waypoints/{canonical_name}_{waypoint_type}.csv'),
                os.path.join(ws_root, f'src/stanley_controller/waypoints/{canonical_name}_raceline.csv'),
                os.path.join(ws_root, f'src/stanley_controller/waypoints/{canonical_name}_centerline.csv'),
                os.path.join(ws_root, 'src/stanley_controller/waypoints/Spielberg_raceline.csv'),
                os.path.join(ws_root, 'src/stanley_controller/waypoints/Spielberg_centerline.csv'),
            ]
            for cand in local_cands:
                if os.path.isfile(cand):
                    active_csv = cand
                    break

        if active_csv is None or not os.path.isfile(active_csv):
            raise FileNotFoundError(
                f"Could not find waypoints CSV for track '{track_name}' in '{track_dir}' or local fallback."
            )

        # Parse CSV format (auto delimiter detection and comment stripping)
        with open(active_csv, 'r') as f:
            first_data_line = ''
            for line in f:
                stripped = line.strip()
                if stripped and not stripped.startswith('#'):
                    first_data_line = stripped
                    break
        sep = ';' if ';' in first_data_line else ','
        data = np.loadtxt(active_csv, delimiter=sep, comments='#')
        if data.ndim == 1:
            data = data.reshape(1, -1)

        cols = data.shape[1]

        # Discover map files first so they can be used for safety validation
        map_yaml = None
        map_png = None
        map_path_no_ext = None
        if os.path.isdir(track_dir):
            yamls = glob.glob(os.path.join(track_dir, '*map.yaml'))
            pngs = glob.glob(os.path.join(track_dir, '*map.png'))
            if yamls:
                map_yaml = yamls[0]
                map_path_no_ext = os.path.splitext(map_yaml)[0]
            if pngs:
                map_png = pngs[0]

        if cols >= 6:
            # Standard 7-column or 6-column raceline: [s, x, y, psi, kappa, vx, (ax)]
            s_arr = data[:, 0]
            waypoints = np.copy(data[:, [1, 2]])
            headings = (data[:, 3] + math.pi) % (2.0 * math.pi) - math.pi
            kappa = np.copy(data[:, 4])
            target_speeds = np.copy(data[:, 5])

            # Safety clearance buffer: check map SDF and nudge waypoints away from obstacles smoothly
            safe_dist = 0.42
            if map_yaml and os.path.isfile(map_yaml) and map_png and os.path.isfile(map_png):
                try:
                    from PIL import Image
                    from scipy.ndimage import distance_transform_edt, gaussian_filter1d

                    with open(map_yaml, 'r') as f:
                        mcfg = yaml.safe_load(f)
                    img = Image.open(map_png).convert('L')
                    map_arr = np.array(img)
                    res = float(mcfg['resolution'])
                    origin = mcfg['origin']

                    free_mask = (map_arr > 200).astype(np.float32)
                    obs_mask = (map_arr <= 200).astype(np.float32)
                    sdf = (distance_transform_edt(free_mask) - distance_transform_edt(obs_mask)) * res

                    # Centerline for safe directional guidance
                    cl_cands = glob.glob(os.path.join(track_dir, '*centerline.csv'))
                    cl_xy = None
                    if cl_cands:
                        sep_cl = ';' if ';' in open(cl_cands[0]).read(200) else ','
                        cl_d = np.loadtxt(cl_cands[0], delimiter=sep_cl, comments='#')
                        cl_xy = cl_d[:, :2]

                    if cl_xy is not None:
                        # 1. Continuous shift field with spatial smoothing
                        for it in range(5):
                            shifts = np.zeros_like(waypoints)
                            for i in range(len(waypoints)):
                                wx, wy = waypoints[i, 0], waypoints[i, 1]
                                px = int((wx - origin[0]) / res)
                                py = int((wy - origin[1]) / res)
                                iy = map_arr.shape[0] - py - 1
                                d = sdf[iy, px] if 0 <= px < map_arr.shape[1] and 0 <= iy < map_arr.shape[0] else -1.0

                                if d < safe_dist:
                                    closest_cl = int(np.argmin((cl_xy[:, 0] - wx)**2 + (cl_xy[:, 1] - wy)**2))
                                    cx, cy = cl_xy[closest_cl]
                                    v_to_cl = np.array([cx - wx, cy - wy])
                                    dist_to_cl = math.hypot(v_to_cl[0], v_to_cl[1])
                                    if dist_to_cl > 1e-3:
                                        shifts[i] = (v_to_cl / dist_to_cl) * (safe_dist - d)

                            shifts[:, 0] = gaussian_filter1d(shifts[:, 0], sigma=4.0, mode='wrap')
                            shifts[:, 1] = gaussian_filter1d(shifts[:, 1], sigma=4.0, mode='wrap')
                            waypoints += shifts * 1.05

                        # 2. Strict obstacle clearance floor check
                        for i in range(len(waypoints)):
                            wx, wy = waypoints[i, 0], waypoints[i, 1]
                            px = int((wx - origin[0]) / res)
                            py = int((wy - origin[1]) / res)
                            iy = map_arr.shape[0] - py - 1
                            d = sdf[iy, px] if 0 <= px < map_arr.shape[1] and 0 <= iy < map_arr.shape[0] else -1.0
                            if d < safe_dist:
                                closest_cl = int(np.argmin((cl_xy[:, 0] - wx)**2 + (cl_xy[:, 1] - wy)**2))
                                cx, cy = cl_xy[closest_cl]
                                v_to_cl = np.array([cx - wx, cy - wy])
                                dist_to_cl = math.hypot(v_to_cl[0], v_to_cl[1])
                                if dist_to_cl > 1e-3:
                                    waypoints[i] += (v_to_cl / dist_to_cl) * (safe_dist - d + 0.02)

                        # Recompute arc length after safety shift
                        diffs = np.diff(waypoints, axis=0)
                        dists = np.hypot(diffs[:, 0], diffs[:, 1])
                        s_arr = np.insert(np.cumsum(dists), 0, 0.0)

                        # Analytic 2D parametric curvature with periodic circular padding
                        N_pad = min(50, len(waypoints) // 4)
                        x_pad = np.pad(waypoints[:, 0], N_pad, mode='wrap')
                        y_pad = np.pad(waypoints[:, 1], N_pad, mode='wrap')

                        dx = np.gradient(x_pad)
                        dy = np.gradient(y_pad)
                        ddx = np.gradient(dx)
                        ddy = np.gradient(dy)

                        denom = (dx**2 + dy**2)**1.5
                        denom = np.where(denom < 1e-6, 1e-6, denom)
                        kappa_pad = (dx * ddy - dy * ddx) / denom
                        kappa = kappa_pad[N_pad:-N_pad]
                        kappa = gaussian_filter1d(kappa, sigma=5.0, mode='wrap')

                        headings_pad = np.arctan2(dy, dx)
                        headings = headings_pad[N_pad:-N_pad]
                        headings = (headings + math.pi) % (2.0 * math.pi) - math.pi
                        sin_h = gaussian_filter1d(np.sin(headings), sigma=3.0, mode='wrap')
                        cos_h = gaussian_filter1d(np.cos(headings), sigma=3.0, mode='wrap')
                        headings = np.arctan2(sin_h, cos_h)

                except Exception as e:
                    print(f"[TrackManager] Warning during map safety clearance check: {e}")

            # Dynamic curvature lookahead speed profiling
            # Anticipates corner apexes ~4 meters in advance so vehicle begins pre-braking on straight
            k_abs = np.abs(kappa)
            N_pts = len(waypoints)
            window_pts = min(20, max(5, N_pts // 20))
            k_lookahead = np.zeros(N_pts)
            for i in range(N_pts):
                k_lookahead[i] = np.max([k_abs[(i + w) % N_pts] for w in range(window_pts)])

            eff_lat_accel = np.clip(lat_accel_max - 1.0 * np.maximum(0.0, k_lookahead - 0.25), 1.6, lat_accel_max)
            v_corner = np.where(k_lookahead > 0.04, np.sqrt(eff_lat_accel / np.maximum(k_lookahead, 1e-4)), max_straight_speed)
            v_corner = np.clip(v_corner, min_corner_speed, max_straight_speed)
            target_speeds = np.where(k_lookahead < 0.04, max_straight_speed, v_corner)

            # Smooth forward-backward longitudinal acceleration/deceleration profiling
            diffs = np.diff(waypoints, axis=0)
            dists = np.hypot(diffs[:, 0], diffs[:, 1])
            closure_step = float(np.hypot(waypoints[0, 0] - waypoints[-1, 0], waypoints[0, 1] - waypoints[-1, 1]))
            if closure_step < 1e-4:
                closure_step = dists[0] if len(dists) > 0 else 0.2

            for _ in range(6):
                # Backward pass (pre-braking into corners ahead of time across lap boundary)
                for i in range(N_pts - 1, -1, -1):
                    i_next = (i + 1) % N_pts
                    ds_step = dists[i] if i < len(dists) else closure_step
                    v_max_brake = math.sqrt(target_speeds[i_next]**2 + 2.0 * a_brake_max * ds_step)
                    if target_speeds[i] > v_max_brake:
                        target_speeds[i] = v_max_brake

                # Forward pass (smooth acceleration out of corners onto straights)
                for i in range(N_pts):
                    i_prev = (i - 1) % N_pts
                    ds_step = dists[i_prev] if i_prev < len(dists) else closure_step
                    v_max_accel = math.sqrt(target_speeds[i_prev]**2 + 2.0 * a_accel_max * ds_step)
                    if target_speeds[i] > v_max_accel:
                        target_speeds[i] = v_max_accel

            if HAS_GAUSSIAN_FILTER:
                target_speeds = gaussian_filter1d(target_speeds, sigma=3.0, mode='wrap')

        elif cols >= 2:
            # 4-column centerline [x, y, w_right, w_left] or 2-column [x, y]
            waypoints = data[:, [0, 1]]
            diffs = np.diff(waypoints, axis=0)
            dists = np.hypot(diffs[:, 0], diffs[:, 1])
            s_arr = np.insert(np.cumsum(dists), 0, 0.0)

            # Gradient-based tangent heading
            dx = np.gradient(waypoints[:, 0])
            dy = np.gradient(waypoints[:, 1])
            headings = np.arctan2(dy, dx)

            # Gradient-based curvature
            dpsi = (np.gradient(headings) + math.pi) % (2.0 * math.pi) - math.pi
            ds = np.gradient(s_arr)
            ds = np.where(ds < 1e-4, 1e-4, ds)
            kappa = dpsi / ds

            # Safe curvature velocity profile with adaptive straight speed and lookahead
            k_abs = np.abs(kappa)
            N_pts = len(waypoints)
            window_pts = min(20, max(5, N_pts // 20))
            k_lookahead = np.zeros(N_pts)
            for i in range(N_pts):
                k_lookahead[i] = np.max([k_abs[(i + w) % N_pts] for w in range(window_pts)])

            eff_lat_accel = np.clip(lat_accel_max - 1.0 * np.maximum(0.0, k_lookahead - 0.25), 1.6, lat_accel_max)
            v_profile = np.where(k_lookahead > 0.04, np.sqrt(eff_lat_accel / np.maximum(k_lookahead, 1e-4)), max_straight_speed)
            target_speeds = np.clip(v_profile, min_corner_speed, max_straight_speed)
            target_speeds = np.where(k_lookahead < 0.04, max_straight_speed, target_speeds)

            # Smooth forward-backward longitudinal acceleration/deceleration profiling
            closure_step = float(np.hypot(waypoints[0, 0] - waypoints[-1, 0], waypoints[0, 1] - waypoints[-1, 1]))
            if closure_step < 1e-4:
                closure_step = dists[0] if len(dists) > 0 else 0.2

            for _ in range(6):
                for i in range(N_pts - 1, -1, -1):
                    i_next = (i + 1) % N_pts
                    ds_step = dists[i] if i < len(dists) else closure_step
                    v_max_brake = math.sqrt(target_speeds[i_next]**2 + 2.0 * a_brake_max * ds_step)
                    if target_speeds[i] > v_max_brake:
                        target_speeds[i] = v_max_brake
                for i in range(N_pts):
                    i_prev = (i - 1) % N_pts
                    ds_step = dists[i_prev] if i_prev < len(dists) else closure_step
                    v_max_accel = math.sqrt(target_speeds[i_prev]**2 + 2.0 * a_accel_max * ds_step)
                    if target_speeds[i] > v_max_accel:
                        target_speeds[i] = v_max_accel

            if HAS_GAUSSIAN_FILTER:
                target_speeds = gaussian_filter1d(target_speeds, sigma=3.0, mode='wrap')
        else:
            raise ValueError(f"Unrecognized waypoint CSV format with {cols} columns: {active_csv}")

        # Compute closed loop track length
        closure_dist = float(np.hypot(waypoints[0, 0] - waypoints[-1, 0],
                                      waypoints[0, 1] - waypoints[-1, 1]))
        track_length = float(s_arr[-1]) + closure_dist

        # Starting pose is first waypoint
        start_pose = (float(waypoints[0, 0]), float(waypoints[0, 1]), float(headings[0]))

        return TrackInfo(
            track_name=canonical_name,
            track_dir=track_dir,
            csv_path=active_csv,
            waypoint_type=actual_type,
            waypoints=waypoints,
            headings=headings,
            s_arr=s_arr,
            kappa=kappa,
            target_speeds=target_speeds,
            track_length=track_length,
            start_pose=start_pose,
            map_yaml_path=map_yaml,
            map_png_path=map_png,
            map_path_no_ext=map_path_no_ext
        )

    @classmethod
    def sync_sim_yaml(cls, track_info: TrackInfo, custom_sim_yaml: Optional[str] = None) -> bool:
        """
        Synchronizes f1tenth_gym_ros sim.yaml configuration with the selected track's map and start pose.
        Updates both source config and installed share config so the simulator loads the exact map.
        """
        ws_roots = [cls.get_workspace_root(), '/sim_ws', '/home/yeswanth/roboracer_ws']
        target_yamls = [custom_sim_yaml]
        for w in ws_roots:
            if w:
                target_yamls.append(os.path.join(w, 'src/f1tenth_gym_ros/config/sim.yaml'))
                target_yamls.append(os.path.join(w, 'install/f1tenth_gym_ros/share/f1tenth_gym_ros/config/sim.yaml'))

        if not track_info.map_path_no_ext or not os.path.isfile(track_info.map_path_no_ext + '.yaml'):
            return False

        updated_any = False
        for ypath in target_yamls:
            if ypath and os.path.isfile(ypath):
                try:
                    with open(ypath, 'r') as f:
                        cfg = yaml.safe_load(f)

                    if 'bridge' in cfg and 'ros__parameters' in cfg['bridge']:
                        p = cfg['bridge']['ros__parameters']
                        p['map_path'] = track_info.map_path_no_ext
                        p['map_img_ext'] = '.png'
                        p['sx'] = track_info.start_pose[0]
                        p['sy'] = track_info.start_pose[1]
                        p['stheta'] = track_info.start_pose[2]

                        with open(ypath, 'w') as f:
                            yaml.dump(cfg, f, default_flow_style=False, sort_keys=False)
                        updated_any = True
                except Exception as e:
                    print(f"[TrackManager] Warning: Failed updating {ypath}: {e}")

        return updated_any
