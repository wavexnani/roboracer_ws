import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


def create_path_marker(waypoints: np.ndarray, frame_id: str = 'map', stamp=None) -> Marker:
    """Create a LINE_STRIP marker of track waypoints for RViz."""
    m = Marker()
    m.header.frame_id = frame_id
    if stamp is not None:
        m.header.stamp = stamp
    m.ns = 'stanley_raceline'
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


def create_target_marker(wpt_x: float, wpt_y: float, frame_id: str = 'map', stamp=None) -> Marker:
    """Create a SPHERE marker at the target waypoint."""
    m = Marker()
    m.header.frame_id = frame_id
    if stamp is not None:
        m.header.stamp = stamp
    m.ns = 'stanley_target'
    m.id = 1
    m.type = Marker.SPHERE
    m.action = Marker.ADD
    m.pose.position.x = float(wpt_x)
    m.pose.position.y = float(wpt_y)
    m.pose.position.z = 0.15
    m.pose.orientation.w = 1.0
    m.scale.x = 0.35
    m.scale.y = 0.35
    m.scale.z = 0.35
    m.color.r = 1.0
    m.color.g = 0.2
    m.color.b = 0.0
    m.color.a = 0.95
    return m
