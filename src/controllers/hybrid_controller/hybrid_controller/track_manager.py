import os
import glob
import math
from dataclasses import dataclass
from typing import Optional, List, Tuple
import numpy as np
import yaml

from .safety_clearance import apply_map_safety_clearance
from .speed_profiler import generate_speed_profile, compute_centerline_geometry


@dataclass
class TrackInfo:
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
    track_width_est: float = 1.2
    sdf_fn: Optional[object] = None


class TrackManager:
    """Discovery, loading, and velocity profiling for F1TENTH maps."""

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
            os.path.join(ws, 'src/simulation/f1tenth_racetracks'),
            os.path.join(ws, 'src/f1tenth_racetracks'),
            '/home/yeswanth/roboracer_ws/src/simulation/f1tenth_racetracks',
            '/home/yeswanth/roboracer_ws/src/f1tenth_racetracks',
            '/sim_ws/src/simulation/f1tenth_racetracks',
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
                os.path.join(ws_root, f'src/controllers/stanley_controller/waypoints/{canonical_name}_{waypoint_type}.csv'),
                os.path.join(ws_root, f'src/controllers/stanley_controller/waypoints/{canonical_name}_raceline.csv'),
                os.path.join(ws_root, f'src/controllers/stanley_controller/waypoints/{canonical_name}_centerline.csv'),
                os.path.join(ws_root, 'src/controllers/stanley_controller/waypoints/Spielberg_raceline.csv'),
                os.path.join(ws_root, 'src/controllers/stanley_controller/waypoints/Spielberg_centerline.csv'),
                os.path.join(ws_root, f'src/stanley_controller/waypoints/{canonical_name}_{waypoint_type}.csv'),
            ]
            for cand in local_cands:
                if os.path.isfile(cand):
                    active_csv = cand
                    break

        if active_csv is None or not os.path.isfile(active_csv):
            raise FileNotFoundError(
                f"Could not find waypoints CSV for track '{track_name}' in '{track_dir}' or local fallback."
            )

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

        sdf_fn_instance = None
        if cols >= 6:
            s_arr = data[:, 0]
            waypoints = np.copy(data[:, [1, 2]])
            headings = (data[:, 3] + math.pi) % (2.0 * math.pi) - math.pi
            kappa = np.copy(data[:, 4])
            target_speeds = np.copy(data[:, 5])

            waypoints, headings, s_arr, kappa, sdf_fn_instance = apply_map_safety_clearance(
                map_yaml, map_png, track_dir, waypoints, headings, s_arr, kappa
            )
            target_speeds = generate_speed_profile(
                waypoints, kappa, lat_accel_max, min_corner_speed,
                max_straight_speed, a_brake_max, a_accel_max
            )
        elif cols >= 2:
            waypoints = data[:, [0, 1]]
            diffs = np.diff(waypoints, axis=0)
            dists = np.hypot(diffs[:, 0], diffs[:, 1])
            s_arr = np.insert(np.cumsum(dists), 0, 0.0)

            headings, kappa = compute_centerline_geometry(waypoints, s_arr)
            target_speeds = generate_speed_profile(
                waypoints, kappa, lat_accel_max, min_corner_speed,
                max_straight_speed, a_brake_max, a_accel_max
            )
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
            # pyrefly: ignore[unbound-name]
            sdf_fn=sdf_fn_instance
        )

    @classmethod
    def sync_sim_yaml(cls, track_info: TrackInfo, custom_sim_yaml: Optional[str] = None) -> bool:
        ws_roots = [cls.get_workspace_root(), '/sim_ws', '/home/yeswanth/roboracer_ws']
        target_yamls = [custom_sim_yaml]
        for w in ws_roots:
            if w:
                target_yamls.extend([
                    os.path.join(w, 'src/simulation/f1tenth_gym_ros/config/sim.yaml'),
                    os.path.join(w, 'src/f1tenth_gym_ros/config/sim.yaml'),
                    os.path.join(w, 'install/f1tenth_gym_ros/share/f1tenth_gym_ros/config/sim.yaml')
                ])

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
