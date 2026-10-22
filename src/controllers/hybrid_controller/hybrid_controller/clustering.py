import math
from dataclasses import dataclass
from typing import List, Optional
import numpy as np


@dataclass
class ObstacleCluster:
    center_x: float
    center_y: float
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    distance: float
    num_points: int


def cluster_corridor_points(
    ranges: List[float],
    angle_min: float = -2.356194,
    angle_max: float = 2.356194,
    cur_x: float = 0.0,
    cur_y: float = 0.0,
    cur_yaw: float = 0.0,
    sdf_fn = None,
    min_cluster_pts: int = 5,
    cluster_dist_thresh: float = 0.20
) -> List[ObstacleCluster]:
    r_arr = np.array(ranges)
    n_beams = len(r_arr)
    if n_beams == 0:
        return []

    angles = np.linspace(angle_min, angle_max, n_beams)
    valid = np.isfinite(r_arr) & (r_arr >= 0.15) & (r_arr <= 8.0)

    r_valid = r_arr[valid]
    th_valid = angles[valid]
    if len(r_valid) < min_cluster_pts:
        return []

    xs = r_valid * np.cos(th_valid)
    ys = r_valid * np.sin(th_valid)

    # Forward vehicle corridor (|ys| <= 0.35m, 0.40m < xs < 9.0m)
    corridor = (xs > 0.40) & (xs < 9.0) & (np.abs(ys) <= 0.35)

    if sdf_fn is not None:
        cos_y = math.cos(cur_yaw)
        sin_y = math.sin(cur_yaw)
        xw = cur_x + xs * cos_y - ys * sin_y
        yw = cur_y + xs * sin_y + ys * cos_y
        sdf_vals = np.array([sdf_fn(xw[i], yw[i]) for i in range(len(xw))])
        corridor = corridor & (sdf_vals >= 0.22)

    c_xs = xs[corridor]
    c_ys = ys[corridor]

    if len(c_xs) < min_cluster_pts:
        return []

    clusters: List[ObstacleCluster] = []
    cur_c_x = [c_xs[0]]
    cur_c_y = [c_ys[0]]

    for i in range(1, len(c_xs)):
        d = math.hypot(c_xs[i] - c_xs[i - 1], c_ys[i] - c_ys[i - 1])
        if d < cluster_dist_thresh:
            cur_c_x.append(c_xs[i])
            cur_c_y.append(c_ys[i])
        else:
            if len(cur_c_x) >= min_cluster_pts:
                arr_x = np.array(cur_c_x)
                arr_y = np.array(cur_c_y)
                cx = float(np.mean(arr_x))
                cy = float(np.mean(arr_y))
                clusters.append(ObstacleCluster(
                    center_x=cx,
                    center_y=cy,
                    min_x=float(np.min(arr_x)),
                    max_x=float(np.max(arr_x)),
                    min_y=float(np.min(arr_y)),
                    max_y=float(np.max(arr_y)),
                    distance=math.hypot(cx, cy),
                    num_points=len(cur_c_x)
                ))
            cur_c_x = [c_xs[i]]
            cur_c_y = [c_ys[i]]

    if len(cur_c_x) >= min_cluster_pts:
        arr_x = np.array(cur_c_x)
        arr_y = np.array(cur_c_y)
        cx = float(np.mean(arr_x))
        cy = float(np.mean(arr_y))
        clusters.append(ObstacleCluster(
            center_x=cx,
            center_y=cy,
            min_x=float(np.min(arr_x)),
            max_x=float(np.max(arr_x)),
            min_y=float(np.min(arr_y)),
            max_y=float(np.max(arr_y)),
            distance=math.hypot(cx, cy),
            num_points=len(cur_c_x)
        ))

    return clusters
