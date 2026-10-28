import os
import glob
import math
import yaml
import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple, List

from .safety_clearance import apply_map_safety_clearance
from .speed_profiler import profile_track_speeds


@dataclass
class TrackInfo:
    """Encapsulates all metadata and trajectory profiles for a racetrack."""
    track_name: str
    track_dir: str
    csv_path: str
    waypoint_type: str
    waypoints: np.ndarray
    headings: np.ndarray
    s_arr: np.ndarray
    kappa: np.ndarray
    target_speeds: np.ndarray
    track_length: float
    start_pose: Tuple[float, float, float]
    map_yaml_path: Optional[str] = None
    map_png_path: Optional[str] = None
    map_path_no_ext: Optional[str] = None


class TrackManager:
    """Manages discovery, loading, and simulator synchronization for racetracks."""

    @staticmethod
    def get_workspace_root() -> str:
        current = os.path.abspath(__file__)
        while current and current != os.path.dirname(current):
            if os.path.isdir(os.path.join(current, 'src', 'simulation', 'f1tenth_racetracks')) or os.path.isdir(os.path.join(current, 'src', 'controllers')) or os.path.isdir(os.path.join(current, 'src', 'f1tenth_racetracks')):
                return current
            if os.path.basename(current) in ('roboracer_ws', 'sim_ws'):
                return current
            current = os.path.dirname(current)
        for cand in ['/home/yeswanth/roboracer_ws', '/sim_ws']:
            if os.path.isdir(cand):
                return cand
        return os.getcwd()

    @classmethod
    def get_racetracks_root(cls) -> str:
        ws_root = cls.get_workspace_root()
        for cand in [
            os.path.join(ws_root, 'src', 'simulation', 'f1tenth_racetracks'),
            os.path.join(ws_root, 'src', 'f1tenth_racetracks'),
            '/home/yeswanth/roboracer_ws/src/simulation/f1tenth_racetracks',
            '/home/yeswanth/roboracer_ws/src/f1tenth_racetracks',
            '/sim_ws/src/simulation/f1tenth_racetracks',
            '/sim_ws/src/f1tenth_racetracks',
        ]:
            if os.path.isdir(cand):
                return cand

        script_dir = os.path.dirname(os.path.abspath(__file__))
        for rel in ['../../../../simulation/f1tenth_racetracks', '../../f1tenth_racetracks']:
            cand2 = os.path.abspath(os.path.join(script_dir, rel))
            if os.path.isdir(cand2):
                return cand2

        return os.path.join(ws_root, 'src', 'simulation', 'f1tenth_racetracks')

    @classmethod
    def list_available_tracks(cls) -> List[str]:
        root = cls.get_racetracks_root()
        if not os.path.isdir(root):
            return []
        return sorted([
            d for d in os.listdir(root)
            if os.path.isdir(os.path.join(root, d)) and not d.startswith('.')
        ])

    @classmethod
    def resolve_track_name(cls, query: str) -> Optional[str]:
        if not query or query.startswith('-'):
            return None
        tracks = cls.list_available_tracks()
        q_norm = query.lower().replace(' ', '').replace('_', '').replace('-', '')
        if not q_norm:
            return None

        for t in tracks:
            if t.lower().replace(' ', '').replace('_', '').replace('-', '') == q_norm:
                return t

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
        """Load waypoints, compute geometric profiles and apply safety clearance."""
        racetracks_root = cls.get_racetracks_root()
        ws_root = cls.get_workspace_root()

        canonical_name = cls.resolve_track_name(track_name) or track_name
        track_dir = os.path.join(racetracks_root, canonical_name)
        active_csv = None
        actual_type = waypoint_type.lower()

        if custom_csv_path and os.path.isfile(custom_csv_path):
            active_csv = custom_csv_path
            actual_type = 'custom'
        elif os.path.isdir(track_dir):
            pref_pattern = '*raceline.csv' if actual_type == 'raceline' else '*centerline.csv'
            alt_pattern = '*centerline.csv' if actual_type == 'raceline' else '*raceline.csv'
            cands = glob.glob(os.path.join(track_dir, pref_pattern))
            if cands:
                active_csv = cands[0]
            else:
                cands_alt = glob.glob(os.path.join(track_dir, alt_pattern))
                if cands_alt:
                    active_csv = cands_alt[0]
                    actual_type = 'centerline' if actual_type == 'raceline' else 'raceline'

        if active_csv is None or not os.path.isfile(active_csv):
            for cand in [
                os.path.join(ws_root, f'src/controllers/stanley_controller/waypoints/{canonical_name}_{waypoint_type}.csv'),
                os.path.join(ws_root, f'src/controllers/stanley_controller/waypoints/{canonical_name}_raceline.csv'),
                os.path.join(ws_root, f'src/controllers/stanley_controller/waypoints/{canonical_name}_centerline.csv'),
                os.path.join(ws_root, 'src/controllers/stanley_controller/waypoints/Spielberg_raceline.csv'),
                os.path.join(ws_root, 'src/controllers/stanley_controller/waypoints/Spielberg_centerline.csv'),
            ]:
                if os.path.isfile(cand):
                    active_csv = cand
                    break

        if active_csv is None or not os.path.isfile(active_csv):
            raise FileNotFoundError(f"Could not find waypoints CSV for track '{track_name}'.")

        with open(active_csv, 'r') as f:
            first_line = next((line.strip() for line in f if line.strip() and not line.strip().startswith('#')), '')
        sep = ';' if ';' in first_line else ','
        data = np.loadtxt(active_csv, delimiter=sep, comments='#')
        if data.ndim == 1:
            data = data.reshape(1, -1)

        cols = data.shape[1]
        map_yaml, map_png, map_path_no_ext = None, None, None
        if os.path.isdir(track_dir):
            yamls = glob.glob(os.path.join(track_dir, '*map.yaml'))
            pngs = glob.glob(os.path.join(track_dir, '*map.png'))
            if yamls:
                map_yaml = yamls[0]
                map_path_no_ext = os.path.splitext(map_yaml)[0]
            if pngs:
                map_png = pngs[0]

        if cols >= 6:
            s_arr = data[:, 0]
            waypoints = np.copy(data[:, [1, 2]])
            headings = (data[:, 3] + math.pi) % (2.0 * math.pi) - math.pi
            kappa = np.copy(data[:, 4])
            target_speeds = np.copy(data[:, 5])

            w_mod, s_mod, k_mod, h_mod = apply_map_safety_clearance(waypoints, track_dir, map_yaml, map_png)
            if s_mod is not None:
                waypoints, s_arr, kappa, headings = w_mod, s_mod, k_mod, h_mod

            diffs = np.diff(waypoints, axis=0)
            dists = np.hypot(diffs[:, 0], diffs[:, 1])
            target_speeds = profile_track_speeds(
                waypoints, s_arr, kappa, dists, lat_accel_max,
                min_corner_speed, max_straight_speed, a_brake_max, a_accel_max
            )
        elif cols >= 2:
            waypoints = data[:, [0, 1]]
            diffs = np.diff(waypoints, axis=0)
            dists = np.hypot(diffs[:, 0], diffs[:, 1])
            s_arr = np.insert(np.cumsum(dists), 0, 0.0)

            dx = np.gradient(waypoints[:, 0])
            dy = np.gradient(waypoints[:, 1])
            headings = np.arctan2(dy, dx)

            dpsi = (np.gradient(headings) + math.pi) % (2.0 * math.pi) - math.pi
            ds = np.maximum(np.gradient(s_arr), 1e-4)
            kappa = dpsi / ds

            target_speeds = profile_track_speeds(
                waypoints, s_arr, kappa, dists, lat_accel_max,
                min_corner_speed, max_straight_speed, a_brake_max, a_accel_max
            )
        else:
            raise ValueError(f"Unrecognized waypoint CSV with {cols} columns: {active_csv}")

        closure_dist = float(np.hypot(waypoints[0, 0] - waypoints[-1, 0], waypoints[0, 1] - waypoints[-1, 1]))
        track_length = float(s_arr[-1]) + closure_dist
        start_pose = (float(waypoints[0, 0]), float(waypoints[0, 1]), float(headings[0]))

        assert kappa is not None
        assert headings is not None
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
        """Update sim.yaml map path and initial pose to match selected track."""
        if not track_info.map_path_no_ext or not os.path.isfile(track_info.map_path_no_ext + '.yaml'):
            return False

        ws_roots = [cls.get_workspace_root(), '/sim_ws', '/home/yeswanth/roboracer_ws']
        target_yamls = [custom_sim_yaml]
        for w in ws_roots:
            if w:
                target_yamls.extend([
                    os.path.join(w, 'src/simulation/f1tenth_gym_ros/config/sim.yaml'),
                    os.path.join(w, 'src/f1tenth_gym_ros/config/sim.yaml'),
                    os.path.join(w, 'install/f1tenth_gym_ros/share/f1tenth_gym_ros/config/sim.yaml')
                ])

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
                    print(f"[TrackManager] Warning updating {ypath}: {e}")

        return updated_any
