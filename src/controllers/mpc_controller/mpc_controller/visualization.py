import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


def create_path_marker(waypoints: np.ndarray, frame_id: str = 'map', stamp=None) -> Marker:
    """Create a LINE_STRIP marker of track waypoints."""
    marker = Marker()
    marker.header.frame_id = frame_id
    if stamp is not None:
        marker.header.stamp = stamp
    marker.ns = 'mpc_reference_path'
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.08
    marker.color.r = 0.0
    marker.color.g = 0.8
    marker.color.b = 1.0
    marker.color.a = 0.7

    for pt in waypoints:
        p = Point()
        p.x = float(pt[0])
        p.y = float(pt[1])
        p.z = 0.02
        marker.points.append(p)

    if len(waypoints) > 0:
        p = Point()
        p.x = float(waypoints[0, 0])
        p.y = float(waypoints[0, 1])
        p.z = 0.02
        marker.points.append(p)

    return marker


def create_target_marker(x: float, y: float, frame_id: str = 'map', stamp=None) -> Marker:
    """Create a SPHERE marker at lookahead target."""
    marker = Marker()
    marker.header.frame_id = frame_id
    if stamp is not None:
        marker.header.stamp = stamp
    marker.ns = 'mpc_target'
    marker.id = 1
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = float(x)
    marker.pose.position.y = float(y)
    marker.pose.position.z = 0.15
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.35
    marker.scale.y = 0.35
    marker.scale.z = 0.35
    marker.color.r = 1.0
    marker.color.g = 0.2
    marker.color.b = 0.0
    marker.color.a = 0.95
    return marker


def create_horizon_marker(pred_x: np.ndarray, pred_y: np.ndarray, frame_id: str = 'map', stamp=None) -> Marker:
    """Create a LINE_STRIP marker for the predicted MPC horizon."""
    marker = Marker()
    marker.header.frame_id = frame_id
    if stamp is not None:
        marker.header.stamp = stamp
    marker.ns = 'mpc_predicted_horizon'
    marker.id = 2
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.12
    marker.color.r = 0.1
    marker.color.g = 1.0
    marker.color.b = 0.1
    marker.color.a = 0.95

    for i in range(len(pred_x)):
        p = Point()
        p.x = float(pred_x[i])
        p.y = float(pred_y[i])
        p.z = 0.05
        marker.points.append(p)

    return marker
