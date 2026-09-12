#!/usr/bin/env python3
"""
TrackManager for Hybrid Controller with Multi-Map Ingestion, SDF Safety Buffer, and Urethane Floor Physics.
Supports all 24 racetracks from f1tenth_racetracks.
Computes arc lengths, 2D parametric curvatures, SDF wall-clearance nudging, and backward-pass pre-braking velocity profiles.
"""

import os
import glob
import math
from dataclasses import dataclass
from typing import Optional, List, Tuple
import numpy as np
import yaml


@dataclass
class TrackInfo:
    """Encapsulates geometric and velocity profile data for a racetrack."""
    track_name: str
    track_dir: str
    csv_path: str
    waypoint_type: str
    waypoints: np.ndarray      # [N, 2] (x, y) [m]
    headings: np.ndarray       # [N] tangent yaw [rad]
    s_arr: np.ndarray          # [N] arc-length distance [m]
    kappa: np.ndarray          # [N] curvature [rad/m]
    target_speeds: np.ndarray  # [N] pre-braked velocity profile [m/s]
    track_length: float        # Total closed-loop length [m]
    start_pose: Tuple[float, float, float]  # (x, y, yaw)
    map_yaml_path: Optional[str]
    map_png_path: Optional[str]
    map_path_no_ext: Optional[str]
    track_width_est: float = 1.2  # Estimated minimum drivable width [m]
    sdf_fn: Optional[object] = None


class TrackManager:
    """Manages universal discovery, loading, and velocity profiling for F1TENTH maps."""

    KNOWN_TRACKS = [
        'Austin', 'BrandsHatch', 'Budapest', 'Catalunya', 'Hockenheim', 'IMS',
        'Levine', 'Melbourne', 'Mexico City', 'Montreal', 'Monza', 'MoscowRaceway',
        'Nuerburgring', 'Oschersleben', 'Sakhir', 'SaoPaulo', 'Sepang',
        'Shanghai', 'Silverstone', 'Sochi', 'Spa', 'Spielberg', 'YasMarina', 'Zandvoort'
    ]

    @classmethod
    def get_workspace_root(cls) -> str:
        for ws in ['/home/yeswanth/roboracer_ws', '/sim_ws']:
            if os.path.isdir(ws):
                return ws
        return os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..'))

    @classmethod
    def get_racetracks_root(cls) -> str:
        ws = cls.get_workspace_root()
        candidates = [
            os.path.join(ws, 'src/f1tenth_racetracks'),
            '/home/yeswanth/roboracer_ws/src/f1tenth_racetracks',
            '/sim_ws/src/f1tenth_racetracks',
        ]
        for c in candidates:
            if os.path.isdir(c):
                return c
        return ''

    @classmethod
    def list_available_tracks(cls) -> List[str]:
        root = cls.get_racetracks_root()
        if not os.path.isdir(root):
            return cls.KNOWN_TRACKS
        found = []
        for name in sorted(os.listdir(root)):
            p = os.path.join(root, name)
            if os.path.isdir(p) and not name.startswith('.'):
                found.append(name)
        return found if found else cls.KNOWN_TRACKS

    @classmethod
    def resolve_track_name(cls, query: str) -> Optional[str]:
        if not query:
            return 'Spielberg'
        q = query.strip().lower()
        if q.startswith('-'):
            return 'Spielberg'
        available = cls.list_available_tracks()
        for t in available:
            if t.lower() == q:
                return t
        for t in available:
            if q in t.lower():
                return t
        return None

    @classmethod
    def load_track(
        cls,
        track_name: str = 'Spielberg',
        waypoint_type: str = 'raceline',
        custom_csv_path: Optional[str] = None,
        max_straight_speed: float = 7.5,
        min_corner_speed: float = 1.8,
        lat_accel_max: float = 2.5,
        a_brake_max: float = 2.0,
        a_accel_max: float = 2.5,
        low_friction_mode: bool = False
    ) -> TrackInfo:
        """
        Loads track with SDF clearance buffering, curvature lookahead, and multi-pass backward pre-braking.
        If low_friction_mode is True (e.g. BEXCO Hall 5A urethane floor), clamps lateral acceleration
        to 2.0 m/s^2 and caps top speed to ensure zero snap oversteer.
        """
        if low_friction_mode:
            lat_accel_max = min(lat_accel_max, 2.0)
            a_brake_max = min(a_brake_max, 1.8)
            a_accel_max = min(a_accel_max, 2.0)
            max_straight_speed = min(max_straight_speed, 5.0)
            min_corner_speed = min(min_corner_speed, 1.6)

        ws_root = cls.get_workspace_root()
        racetracks_root = cls.get_racetracks_root()
        canonical_name = cls.resolve_track_name(track_name) or track_name
        track_dir = os.path.join(racetracks_root, canonical_name)
        active_csv = None
        actual_type = waypoint_type.lower()

        if custom_csv_path and os.path.isfile(custom_csv_path):
            active_csv = custom_csv_path
            actual_type = 'custom'
        elif os.path.isdir(track_dir):
            if actual_type == 'raceline':
                cand = glob.glob(os.path.join(track_dir, '*raceline.csv'))
                if cand:
                    active_csv = cand[0]
                else:
                    cand = glob.glob(os.path.join(track_dir, '*centerline.csv'))
                    if cand:
                        active_csv = cand[0]
                        actual_type = 'centerline'
            else:
                cand = glob.glob(os.path.join(track_dir, '*centerline.csv'))
                if cand:
                    active_csv = cand[0]
                    actual_type = 'centerline'
                else:
                    cand = glob.glob(os.path.join(track_dir, '*raceline.csv'))
                    if cand:
                        active_csv = cand[0]
                        actual_type = 'raceline'

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

        # Parse CSV
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

        # Discover map files
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
            # Standard raceline: [s, x, y, psi, kappa, vx, (ax)]
            s_arr = data[:, 0]
            waypoints = np.copy(data[:, [1, 2]])
            headings = (data[:, 3] + math.pi) % (2.0 * math.pi) - math.pi
            kappa = np.copy(data[:, 4])
            target_speeds = np.copy(data[:, 5])

            # Safety clearance buffer: check map SDF and nudge waypoints away from obstacles smoothly
            safe_dist = 0.42
            sdf_fn_instance = None
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

                    def get_sdf_val(x_w, y_w):
                        px = int((x_w - origin[0]) / res)
                        py = int((y_w - origin[1]) / res)
                        iy = map_arr.shape[0] - py - 1
                        if 0 <= px < map_arr.shape[1] and 0 <= iy < map_arr.shape[0]:
                            return float(sdf[iy, px])
                        return -1.0
                    sdf_fn_instance = get_sdf_val

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

                            shifts[:, 0] = gaussian_filter1d(shifts[:, 0], sigma=2.0, mode='wrap')
                            shifts[:, 1] = gaussian_filter1d(shifts[:, 1], sigma=2.0, mode='wrap')
                            waypoints += shifts * 1.20

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
                        kappa = gaussian_filter1d(kappa, sigma=2.5, mode='wrap')

                        headings_pad = np.arctan2(dy, dx)
                        headings = headings_pad[N_pad:-N_pad]
                        headings = (headings + math.pi) % (2.0 * math.pi) - math.pi

                except Exception as e:
                    print(f"[TrackManager] Warning during map safety clearance check: {e}")

            # Dynamic curvature lookahead speed profiling
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
            for _ in range(4):
                # Backward pass
                for i in range(N_pts - 1, -1, -1):
                    i_next = (i + 1) % N_pts
                    ds_step = dists[i] if i < len(dists) else dists[-1]
                    v_max_brake = math.sqrt(target_speeds[i_next]**2 + 2.0 * a_brake_max * ds_step)
                    if target_speeds[i] > v_max_brake:
                        target_speeds[i] = v_max_brake

                # Forward pass
                for i in range(N_pts):
                    i_prev = (i - 1) % N_pts
                    ds_step = dists[i_prev] if i_prev < len(dists) else dists[0]
                    v_max_accel = math.sqrt(target_speeds[i_prev]**2 + 2.0 * a_accel_max * ds_step)
                    if target_speeds[i] > v_max_accel:
                        target_speeds[i] = v_max_accel

        elif cols >= 2:
            waypoints = data[:, [0, 1]]
            diffs = np.diff(waypoints, axis=0)
            dists = np.hypot(diffs[:, 0], diffs[:, 1])
            s_arr = np.insert(np.cumsum(dists), 0, 0.0)

            dx = np.gradient(waypoints[:, 0])
            dy = np.gradient(waypoints[:, 1])
            headings = np.arctan2(dy, dx)

            dpsi = (np.gradient(headings) + math.pi) % (2.0 * math.pi) - math.pi
            ds = np.gradient(s_arr)
            ds = np.where(ds < 1e-4, 1e-4, ds)
            kappa = dpsi / ds

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

            for _ in range(4):
                for i in range(N_pts - 1, -1, -1):
                    i_next = (i + 1) % N_pts
                    ds_step = dists[i] if i < len(dists) else dists[-1]
                    v_max_brake = math.sqrt(target_speeds[i_next]**2 + 2.0 * a_brake_max * ds_step)
                    if target_speeds[i] > v_max_brake:
                        target_speeds[i] = v_max_brake
                for i in range(N_pts):
                    i_prev = (i - 1) % N_pts
                    ds_step = dists[i_prev] if i_prev < len(dists) else dists[0]
                    v_max_accel = math.sqrt(target_speeds[i_prev]**2 + 2.0 * a_accel_max * ds_step)
                    if target_speeds[i] > v_max_accel:
                        target_speeds[i] = v_max_accel
        else:
            raise ValueError(f"Unrecognized waypoint CSV format with {cols} columns: {active_csv}")

        closure_dist = float(np.hypot(waypoints[0, 0] - waypoints[-1, 0],
                                      waypoints[0, 1] - waypoints[-1, 1]))
        track_length = float(s_arr[-1]) + closure_dist
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
            map_path_no_ext=map_path_no_ext,
            sdf_fn=sdf_fn_instance
        )

    @classmethod
    def sync_sim_yaml(cls, track_info: TrackInfo, custom_sim_yaml: Optional[str] = None) -> bool:
        """Synchronizes f1tenth_gym_ros sim.yaml configuration with selected track and start pose."""
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
