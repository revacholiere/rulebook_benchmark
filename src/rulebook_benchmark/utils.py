import numpy as np
import math
import shapely


def normalize_angle(angle):
    while angle > math.pi:
        angle -= math.tau
    while angle < -math.pi:
        angle += math.tau
    assert -math.pi <= angle <= math.pi
    return angle

def angle_between(v1, v2):
    x, y = v1
    x2, y2 = v2
    return normalize_angle(math.atan2(y2, x2) - math.atan2(y, x))


def project_vector(v1, v2): #project v1 onto v2
    v2_norm = np.linalg.norm(v2)
    if v2_norm == 0:
        return np.zeros_like(v1)
    return (np.dot(v1, v2) / v2_norm**2) * v2


def normalize_vector(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm

def polygon_distance(x_state, y_state):
    x_polygon = x_state.polygon
    y_polygon = y_state.polygon
    return shapely.distance(x_polygon, y_polygon)

def intersects(x_state, y_state):
    x_polygon = x_state.polygon
    y_polygon = y_state.polygon
    return shapely.intersects(x_polygon, y_polygon)

def in_proximity(ego_state, object_states, threshold):
        if len(object_states) == 0:
            return []
        ego = ego_state.object
        radius = math.sqrt(ego.width ** 2 + ego.length ** 2) + threshold
        ego_pos = ego_state.position
        adv_positions = np.array([v.position for v in object_states])
        adv_radii = np.array([math.sqrt(v.object.width ** 2 + v.object.length ** 2) for v in object_states])
        distances = np.linalg.norm(adv_positions - ego_pos, axis=1)
        mask = distances < (radius + adv_radii)
        return [v for v, m in zip(object_states, mask) if m]

    
    
    


def project_polygon(vertices, axis):
    """Project polygon onto axis and return [min, max]."""
    dots = [np.dot(v, axis) for v in vertices]
    return min(dots), max(dots)

def continuous_ttc(ego_vertices, adv_vertices, v_rel, threshold):
    """
    Returns earliest collision time in [0, threshold] or None if no collision.
    """
    # Build candidate separating axes = edge normals of both polygons
    axes = []
    for poly in [ego_vertices, adv_vertices]:
        for i in range(len(poly)):
            p1, p2 = np.array(poly[i]), np.array(poly[(i+1) % len(poly)])
            edge = p2 - p1
            normal = np.array([-edge[1], edge[0]])  # perpendicular
            if np.linalg.norm(normal) > 1e-9:
                normal = normal / np.linalg.norm(normal)
                axes.append(normal)

    t_enter, t_exit = 0.0, threshold

    for axis in axes:
        # Project ego (stationary)
        e_min, e_max = project_polygon(ego_vertices, axis)
        # Project adv at t=0
        a_min0, a_max0 = project_polygon(adv_vertices, axis)
        v_proj = np.dot(v_rel, axis)

        if abs(v_proj) < 1e-12:
            # No relative motion along this axis: must overlap at t=0
            if a_max0 < e_min or e_max < a_min0:
                return None
            else:
                continue

        # Times when projections *start* and *stop* overlapping
        t0 = (e_min - a_max0) / v_proj
        t1 = (e_max - a_min0) / v_proj
        t_axis_enter, t_axis_exit = min(t0, t1), max(t0, t1)

        # Shrink global window
        t_enter = max(t_enter, t_axis_enter)
        t_exit = min(t_exit, t_axis_exit)
        if t_enter > t_exit:
            return None  # separating axis → no collision

    if t_enter < 0:
        t_enter = 0
    return t_enter if t_enter <= threshold else None

    


