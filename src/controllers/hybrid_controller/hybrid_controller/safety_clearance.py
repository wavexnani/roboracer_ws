import os
import glob
import math
from typing import Optional, Tuple
import numpy as np
import yaml


def apply_map_safety_clearance(
    map_yaml: Optional[str],
    map_png: Optional[str],
    track_dir: str,
    waypoints: np.ndarray,
    headings: np.ndarray,
    s_arr: np.ndarray,
    kappa: np.ndarray,
    safe_dist: float = 0.42
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, Optional[object]]:
    """Checks map SDF and nudges waypoints away from obstacles smoothly."""
    sdf_fn_instance = None
    if not (map_yaml and os.path.isfile(map_yaml) and map_png and os.path.isfile(map_png)):
        return waypoints, headings, s_arr, kappa, sdf_fn_instance

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
            with open(cl_cands[0], 'r') as f_cl:
                sep_cl = ';' if ';' in f_cl.read(200) else ','
            cl_d = np.loadtxt(cl_cands[0], delimiter=sep_cl, comments='#')
            cl_xy = cl_d[:, :2]

        if cl_xy is not None:
            # 1. Continuous shift field with spatial smoothing
            for _ in range(5):
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

    return waypoints, headings, s_arr, kappa, sdf_fn_instance
