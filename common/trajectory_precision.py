"""Numerical cleanup for the Validator's float32 coordinate representation."""
import math

import numpy as np


def precision_distance(points, epsilon):
    magnitude = max(1., max(abs(float(v)) for point in points for v in point))
    return max(2 * epsilon, 64 * float(np.spacing(np.float32(magnitude))))


def retained_indices(points, threshold):
    """Preserve both endpoints and eliminate only near-coincident interior points."""
    kept = [0]
    for index in range(1, len(points) - 1):
        if math.dist(points[kept[-1]], points[index]) > threshold:
            kept.append(index)
    while len(kept) > 1 and math.dist(points[kept[-1]], points[-1]) <= threshold:
        kept.pop()
    kept.append(len(points) - 1)
    return kept


def clean_deposition_points(points, config):
    threshold = precision_distance(points, config.collision.geometry_epsilon_mm)
    cleaned = [points[i] for i in retained_indices(points, threshold)]
    if len(cleaned) == 2 and math.dist(*cleaned) <= threshold:
        return []
    return cleaned
