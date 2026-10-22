import math
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


def create_raceline_marker(waypoints: np.ndarray, frame_id: str = 'map', stamp=None) -> Marker:
    """Create LINE_STRIP marker for the static raceline."""
    m = Marker()
    m.header.frame_id = frame_id
    if stamp is not None:
        m.header.stamp = stamp
    m.ns = 'raceline'
    m.id = 0
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.scale.x = 0.08
    m.color.r = 0.0
    m.color.g = 0.8
    m.color.b = 1.0
    m.color.a = 0.7

    for pt in waypoints:
        p = Point()
        p.x = float(pt[0])
        p.y = float(pt[1])
        p.z = 0.02
        m.points.append(p)

    if len(waypoints) > 0:
        p = Point()
        p.x = float(waypoints[0, 0])
        p.y = float(waypoints[0, 1])
        p.z = 0.02
        m.points.append(p)

    return m


def create_regime_marker(regime, speed: float, steering: float, x: float, y: float, frame_id: str = 'map', stamp=None) -> Marker:
    """Create TEXT_VIEW_FACING marker indicating active driving regime and speed/steer."""
    m = Marker()
    m.header.frame_id = frame_id
    if stamp is not None:
        m.header.stamp = stamp
    m.ns = 'active_regime'
    m.id = 1
    m.type = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.pose.position.x = float(x)
    m.pose.position.y = float(y)
    m.pose.position.z = 0.60
    m.scale.z = 0.30

    regime_name = regime.value if hasattr(regime, 'value') else str(regime)
    m.text = f"[{regime_name}]\nV: {speed:.1f} m/s | Steer: {math.degrees(steering):.1f}°"

    color_map = {
        'WALL_DAMPED': (1.0, 0.65, 0.0, 1.0),
        'OBSTACLE_BYPASS': (1.0, 0.0, 0.8, 1.0),
        'HIGH_SPEED_STRAIGHT': (0.0, 1.0, 0.3, 1.0),
        'CORNER_APEX': (0.2, 0.7, 1.0, 1.0),
        'STANLEY_RECOVERY': (1.0, 0.1, 0.1, 1.0),
    }
    r, g, b, a = color_map.get(regime_name, (1.0, 1.0, 1.0, 1.0))
    m.color.r = r
    m.color.g = g
    m.color.b = b
    m.color.a = a
    return m


def create_horizon_marker(pred_trajectory: np.ndarray, frame_id: str = 'map', stamp=None) -> Marker:
    """Create LINE_STRIP marker for predicted trajectory."""
    m = Marker()
    m.header.frame_id = frame_id
    if stamp is not None:
        m.header.stamp = stamp
    m.ns = 'predicted_horizon'
    m.id = 2
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.scale.x = 0.09
    m.color.r = 0.0
    m.color.g = 1.0
    m.color.b = 0.1
    m.color.a = 0.9

    for k in range(len(pred_trajectory)):
        p = Point()
        p.x = float(pred_trajectory[k, 0])
        p.y = float(pred_trajectory[k, 1])
        p.z = 0.03
        m.points.append(p)

    return m
