import os
import glob
import math
import yaml
import numpy as np
from typing import Optional, Tuple

try:
    from scipy.ndimage import distance_transform_edt, gaussian_filter1d
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

    def gaussian_filter1d(input, *args, **kwargs):
        return input

    def distance_transform_edt(input, *args, **kwargs):
        return input


def apply_map_safety_clearance(
    waypoints: np.ndarray,
    track_dir: str,
    map_yaml: Optional[str],
    map_png: Optional[str],
    safe_dist: float = 0.42
) -> Tuple[np.ndarray, Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
    """Check map signed distance field and nudge waypoints away from obstacles."""
    if not (map_yaml and os.path.isfile(map_yaml) and map_png and os.path.isfile(map_png)):
        return waypoints, None, None, None

    try:
        from PIL import Image

        with open(map_yaml, 'r') as f:
            mcfg = yaml.safe_load(f)
        img = Image.open(map_png).convert('L')
        map_arr = np.array(img)
        res = float(mcfg['resolution'])
        origin = mcfg['origin']

        free_mask = (map_arr > 200).astype(np.float32)
        obs_mask = (map_arr <= 200).astype(np.float32)
        sdf = (distance_transform_edt(free_mask) - distance_transform_edt(obs_mask)) * res

        cl_cands = glob.glob(os.path.join(track_dir, '*centerline.csv'))
        cl_xy = None
        if cl_cands:
            sep_cl = ';' if ';' in open(cl_cands[0]).read(200) else ','
            cl_d = np.loadtxt(cl_cands[0], delimiter=sep_cl, comments='#')
            cl_xy = cl_d[:, :2]

        if cl_xy is None:
            return waypoints, None, None, None

        # Shift field with spatial smoothing
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

            shifts[:, 0] = gaussian_filter1d(shifts[:, 0], sigma=4.0, mode='wrap')
            shifts[:, 1] = gaussian_filter1d(shifts[:, 1], sigma=4.0, mode='wrap')
            waypoints += shifts * 1.05

        # Strict obstacle clearance floor check
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

        # Recompute geometry
        diffs = np.diff(waypoints, axis=0)
        dists = np.hypot(diffs[:, 0], diffs[:, 1])
        s_arr = np.insert(np.cumsum(dists), 0, 0.0)

        n_pad = min(50, len(waypoints) // 4)
        x_pad = np.pad(waypoints[:, 0], n_pad, mode='wrap')
        y_pad = np.pad(waypoints[:, 1], n_pad, mode='wrap')

        dx = np.gradient(x_pad)
        dy = np.gradient(y_pad)
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)

        denom = np.maximum((dx**2 + dy**2)**1.5, 1e-6)
        kappa_pad = (dx * ddy - dy * ddx) / denom
        kappa = gaussian_filter1d(kappa_pad[n_pad:-n_pad], sigma=5.0, mode='wrap')

        headings_pad = np.arctan2(dy, dx)
        headings = (headings_pad[n_pad:-n_pad] + math.pi) % (2.0 * math.pi) - math.pi
        sin_h = gaussian_filter1d(np.sin(headings), sigma=3.0, mode='wrap')
        cos_h = gaussian_filter1d(np.cos(headings), sigma=3.0, mode='wrap')
        headings = np.arctan2(sin_h, cos_h)

        return waypoints, s_arr, kappa, headings

    except Exception as e:
        print(f"[TrackManager] Warning during map safety clearance check: {e}")
        return waypoints, None, None, None
